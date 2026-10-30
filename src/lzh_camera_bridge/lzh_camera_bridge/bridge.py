"""Publish the existing PC2 JPEG streams as standard ROS 2 camera interfaces."""
from copy import deepcopy
from array import array
import os
from pathlib import Path
import time

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from sensor_msgs.srv import SetCameraInfo
import yaml

from .core import JpegReceiver, split_frame, validate_calibration, store_calibration


def require_domain_42():
    domain = os.environ.get('ROS_DOMAIN_ID')
    if domain != '42':
        raise RuntimeError(f'ROS_DOMAIN_ID must be 42; got {domain!r}')
    for variable in (
        'AMENT_PREFIX_PATH', 'CMAKE_PREFIX_PATH', 'COLCON_PREFIX_PATH',
        'PYTHONPATH', 'LD_LIBRARY_PATH'
    ):
        for entry in os.environ.get(variable, '').split(os.pathsep):
            if any(marker in entry for marker in ('g1d_ros2_ws', 'g1d_ros2_yqqh', 'unitree_ros_official')):
                raise RuntimeError(f'{variable} contains forbidden overlay path: {entry}')


def as_yaml(info, name):
    matrix = lambda values, rows, cols: {'rows': rows, 'cols': cols, 'data': [float(x) for x in values]}
    return {'image_width': info.width, 'image_height': info.height, 'camera_name': name,
            'distortion_model': info.distortion_model,
            'distortion_coefficients': matrix(info.d, 1, len(info.d)),
            'camera_matrix': matrix(info.k, 3, 3),
            'rectification_matrix': matrix(info.r, 3, 3),
            'projection_matrix': matrix(info.p, 3, 4)}


def from_yaml(data):
    validate_calibration(data)
    info = CameraInfo()
    info.width = int(data['image_width'])
    info.height = int(data['image_height'])
    info.distortion_model = data['distortion_model']
    info.d = [float(x) for x in data['distortion_coefficients']['data']]
    for field, key in [('k', 'camera_matrix'), ('r', 'rectification_matrix'), ('p', 'projection_matrix')]:
        setattr(info, field, [float(x) for x in data[key]['data']])
    return info


class CameraBridge(Node):
    def __init__(self):
        super().__init__('lzh_camera_bridge')
        defaults = {'server_ip': '192.168.123.164', 'topic_base': '/g1d/cameras',
                    'head_port': 55555, 'left_wrist_port': 55557, 'right_wrist_port': 55556,
                    'head_layout': 'sbs', 'head_order': 'lr', 'publish_rate': 15.0,
                    'calibration_dir': '~/.ros/g1d_camera_calibration'}
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        value = lambda name: self.get_parameter(name).value
        self.base = str(value('topic_base')).rstrip('/')
        self.layout = value('head_layout')
        self.order = value('head_order')
        rate = float(value('publish_rate'))
        if self.layout not in ('sbs', 'mono') or self.order not in ('lr', 'rl') or not 0 < rate <= 60:
            raise ValueError('Check head_layout, head_order and publish_rate (0 < rate <= 60)')
        cv2.setNumThreads(1)
        self.calibration_dir = Path(value('calibration_dir')).expanduser()
        self.names = (['head_left', 'head_right'] if self.layout == 'sbs' else ['head']) + ['left_wrist', 'right_wrist']
        self.images, self.infos, self.calibrations, self.sizes = {}, {}, {}, {}
        self._camera_info_services = []
        self.warned_sizes = set()
        for name in self.names:
            prefix = f'{self.base}/{name}'
            self.images[name] = self.create_publisher(Image, prefix + '/image_raw', qos_profile_sensor_data)
            self.infos[name] = self.create_publisher(CameraInfo, prefix + '/camera_info', qos_profile_sensor_data)
            self._camera_info_services.append(self.create_service(SetCameraInfo, prefix + '/set_camera_info',
                lambda request, response, camera=name: self.set_info(camera, request, response)))
            path = self.calibration_dir / (name + '.yaml')
            if path.exists():
                try:
                    self.calibrations[name] = from_yaml(yaml.safe_load(path.read_text()))
                    self.get_logger().info(f'Loaded calibration: {path}')
                except Exception as exc:
                    self.get_logger().warning(f'Ignoring invalid calibration {path}: {exc}')
        ports = {name: int(value(name + '_port')) for name in ('head', 'left_wrist', 'right_wrist')}
        self.receiver = JpegReceiver(value('server_ip'), ports,
                                     stamp_ns=lambda: self.get_clock().now().nanoseconds)
        self.last_sequence = {}
        self._published_counts = {name: 0 for name in self.names}
        self._previous_rx_counts = {}
        self._previous_pub_counts = dict(self._published_counts)
        self._previous_metrics_time = time.monotonic()
        self.stale = set()
        self.started = time.monotonic()
        self.receiver.start()
        self.create_timer(1.0 / rate, self.publish_latest)
        self.create_timer(2.0, self.check_streams)
        self.create_timer(5.0, self.report_rates)
        self.get_logger().info(f'Physical camera mapping: {ports}; publish limit {rate:g} Hz')
        self.get_logger().warning('Image header stamps are bridge receive times, NOT exposure timestamps. No camera TF is inferred.')

    def publish_latest(self):
        frames, _ = self.receiver.snapshot()
        for stream, frame in frames.items():
            if frame.sequence == self.last_sequence.get(stream):
                continue
            self.last_sequence[stream] = frame.sequence
            try:
                views = split_frame(stream, frame.image, self.layout, self.order)
            except ValueError as exc:
                self.get_logger().error(f'{stream}: {exc}')
                continue
            for name, image in views.items():
                height, width = image.shape[:2]
                self.sizes[name] = (width, height)
                msg = Image()
                msg.header.stamp.sec = frame.stamp_ns // 1_000_000_000
                msg.header.stamp.nanosec = frame.stamp_ns % 1_000_000_000
                msg.header.frame_id = name + '_camera_optical_frame'
                msg.height, msg.width = height, width
                msg.encoding, msg.is_bigendian, msg.step = 'bgr8', 0, width * 3
                # Humble's generated uint8[] setter accepts array('B') directly.
                # Passing bytes instead invokes Python validation for every byte.
                payload = array('B')
                payload.frombytes(image.tobytes())
                msg.data = payload
                info = self.calibrations.get(name)
                if info is not None and (info.width, info.height) != (width, height):
                    if name not in self.warned_sizes:
                        self.get_logger().warning(f'{name}: calibration size mismatch; publishing uncalibrated CameraInfo')
                        self.warned_sizes.add(name)
                    info = None
                info = deepcopy(info) if info is not None else CameraInfo()
                info.width, info.height, info.header = width, height, deepcopy(msg.header)
                self.images[name].publish(msg)
                self.infos[name].publish(info)
                self._published_counts[name] += 1

    def report_rates(self):
        frames, _ = self.receiver.snapshot()
        now = time.monotonic()
        elapsed = max(now - self._previous_metrics_time, 1e-9)
        received, published = [], []
        for name in self.receiver.ports:
            count = frames[name].sequence if name in frames else 0
            rate = (count - self._previous_rx_counts.get(name, 0)) / elapsed
            received.append(f'{name}={rate:.1f}')
            self._previous_rx_counts[name] = count
        for name, count in self._published_counts.items():
            rate = (count - self._previous_pub_counts[name]) / elapsed
            published.append(f'{name}={rate:.1f}')
        self.get_logger().info('[FPS] rx_JPEG ' + ' '.join(received) + ' | pub_ROS ' + ' '.join(published))
        self._previous_pub_counts = dict(self._published_counts)
        self._previous_metrics_time = now

    def check_streams(self):
        frames, error = self.receiver.snapshot()
        if error:
            self.get_logger().error(f'JPEG receiver stopped: {error}')
        now = time.monotonic()
        for name in self.receiver.ports:
            frame = frames.get(name)
            stale = frame is None or now - frame.received_monotonic > 2.0
            if stale and name not in self.stale:
                self.get_logger().warning(f'{name}: no recent frames; check PC2 power, image service and network')
                self.stale.add(name)
            elif not stale and name in self.stale:
                self.get_logger().info(f'{name}: frames resumed')
                self.stale.remove(name)

    def set_info(self, name, request, response):
        try:
            if name not in self.sizes:
                raise ValueError('No image received yet')
            data = as_yaml(request.camera_info, name)
            path = store_calibration(self.calibration_dir, name, data, self.sizes[name])
            self.calibrations[name] = from_yaml(data)
            response.success, response.status_message = True, f'Saved on this workstation: {path}'
            self.get_logger().info(response.status_message)
        except Exception as exc:
            response.success, response.status_message = False, str(exc)
        return response

    def destroy_node(self):
        self.receiver.close()
        return super().destroy_node()


def main(args=None):
    require_domain_42()
    rclpy.init(args=args)
    node = None
    try:
        node = CameraBridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
