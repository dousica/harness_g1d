"""Local ROS 2 mosaic viewer; S saves fresh unannotated images, Q quits."""
from datetime import datetime, timezone
from collections import deque
import json
import os
from pathlib import Path
import time
import threading

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor, ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from .core import PORTS, rate_from_times
from .bridge import require_domain_42


class CameraViewer(Node):
    def __init__(self):
        super().__init__('lzh_camera_viewer')
        for key, value in {'topic_base': '/g1d/cameras', 'head_layout': 'sbs',
                           'capture_dir': '~/g1d_vision_data', 'display_scale': 0.75,
                           'viewer_rate': 20.0,
                           'head_port': 55555, 'left_wrist_port': 55557, 'right_wrist_port': 55556}.items():
            self.declare_parameter(key, value)
        self.base = self.get_parameter('topic_base').value.rstrip('/')
        self.layout = self.get_parameter('head_layout').value
        self.scale = float(self.get_parameter('display_scale').value)
        self.viewer_rate = float(self.get_parameter('viewer_rate').value)
        if not 1.0 <= self.viewer_rate <= 60.0:
            raise ValueError('viewer_rate must be between 1 and 60')
        if not 0.1 <= self.scale <= 2.0:
            raise ValueError('display_scale must be between 0.1 and 2.0')
        self.output = Path(self.get_parameter('capture_dir').value).expanduser()
        self.ports = {name: int(self.get_parameter(name + '_port').value) for name in PORTS}
        self.names = (['head_left', 'head_right'] if self.layout == 'sbs' else ['head']) + ['left_wrist', 'right_wrist']
        self.latest = {}
        self._frame_lock = threading.Lock()
        self._arrival_times = {name: deque(maxlen=60) for name in self.names}
        self.subscriptions_owned = [self.create_subscription(Image, f'{self.base}/{name}/image_raw',
            lambda msg, camera=name: self.receive(camera, msg), qos_profile_sensor_data) for name in self.names]

    def receive(self, name, msg):
        if msg.encoding != 'bgr8' or msg.step < msg.width * 3:
            self.get_logger().error(f'{name}: expected bgr8 image')
            return
        image = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)
        image = image[:, :msg.width * 3].reshape(msg.height, msg.width, 3).copy()
        stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        now = time.monotonic()
        with self._frame_lock:
            self.latest[name] = (image, stamp, now)
            self._arrival_times[name].append(now)

    def snapshot(self):
        with self._frame_lock:
            frames = dict(self.latest)
            rates = {name: rate_from_times(times) for name, times in self._arrival_times.items()}
        return frames, rates

    def panel(self, name, frames, rates):
        item = frames.get(name)
        image = np.zeros((480, 640, 3), np.uint8) if item is None else item[0].copy()
        age = float('inf') if item is None else time.monotonic() - item[2]
        stale = age > 1.0
        rate = 0.0 if stale else rates.get(name, 0.0)
        text = f'{name} ROS {rate:.1f}Hz' + (' [NO RECENT IMAGE]' if stale else '')
        cv2.putText(image, text, (12, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65,
                    (0, 0, 255) if stale else (0, 255, 0), 2)
        return cv2.resize(image, (640, 480))

    def show(self):
        frames, rates = self.snapshot()
        if self.layout == 'sbs':
            top = np.hstack([self.panel('head_left', frames, rates), self.panel('head_right', frames, rates)])
        else:
            top = cv2.resize(self.panel('head', frames, rates), (1280, 480))
        bottom = np.hstack([self.panel('left_wrist', frames, rates), self.panel('right_wrist', frames, rates)])
        mosaic = np.vstack([top, bottom])
        cv2.imshow('G1-D cameras | S: save originals | Q: quit',
                   cv2.resize(mosaic, None, fx=self.scale, fy=self.scale))

    def save(self):
        frames, _ = self.snapshot()
        now = time.monotonic()
        if any(name not in frames or now - frames[name][2] > 1.0 for name in self.names):
            self.get_logger().warning('Snapshot skipped: a camera is missing or stale')
            return
        if self.layout == 'sbs' and frames['head_left'][1] != frames['head_right'][1]:
            self.get_logger().warning('Snapshot skipped: head images are from different received frames; press S again')
            return
        identifier = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')
        folder = self.output / identifier
        folder.mkdir(parents=True, exist_ok=False)
        metadata = {'timestamp_kind': 'workstation_bridge_receive_time',
                    'hardware_synchronized': False, 'physical_stream_ports': self.ports,
                    'images': {}}
        for name in self.names:
            image, stamp, _ = frames[name]
            if not cv2.imwrite(str(folder / (name + '.png')), image):
                raise OSError(f'Unable to save {name}')
            metadata['images'][name] = {'receive_stamp_ns': stamp,
                                      'width': image.shape[1], 'height': image.shape[0]}
        stamps = [item['receive_stamp_ns'] for item in metadata['images'].values()]
        metadata['max_receive_stamp_difference_ms'] = (max(stamps) - min(stamps)) / 1e6
        (folder / 'metadata.json').write_text(json.dumps(metadata, indent=2), encoding='utf-8')
        self.get_logger().info(f'Saved unannotated full-resolution images: {folder}')


def main(args=None):
    require_domain_42()
    rclpy.init(args=args)
    node = None
    executor = None
    spin_thread = None
    try:
        if not os.environ.get('DISPLAY') and not os.environ.get('WAYLAND_DISPLAY'):
            raise RuntimeError('Run camera_viewer in the Ubuntu workstation desktop terminal, not a headless SSH session')
        cv2.setNumThreads(1)
        node = CameraViewer()
        executor = SingleThreadedExecutor()
        executor.add_node(node)

        def spin_callbacks():
            try:
                executor.spin()
            except ExternalShutdownException:
                pass

        spin_thread = threading.Thread(target=spin_callbacks, daemon=True, name='g1d-viewer-ROS')
        spin_thread.start()
        while rclpy.ok():
            started = time.monotonic()
            node.show()
            key = cv2.waitKey(1) & 0xff
            if key == ord('q'):
                break
            if key == ord('s'):
                node.save()
            remaining = 1.0 / node.viewer_rate - (time.monotonic() - started)
            if remaining > 0.0:
                time.sleep(remaining)
    except KeyboardInterrupt:
        pass
    finally:
        if executor is not None:
            executor.shutdown(timeout_sec=2.0)
        if spin_thread is not None:
            spin_thread.join(timeout=2.0)
        cv2.destroyAllWindows()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
