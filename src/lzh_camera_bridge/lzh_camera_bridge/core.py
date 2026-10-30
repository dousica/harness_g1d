"""JPEG transport and calibration validation, independent of ROS."""
from dataclasses import dataclass
import math
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import zmq
import yaml

# Physical sides verified by covering the cameras on 2026-10-22.
PORTS = {'head': 55555, 'left_wrist': 55557, 'right_wrist': 55556}


def rate_from_times(times):
    if len(times) < 2:
        return 0.0
    elapsed = times[-1] - times[0]
    return (len(times) - 1) / elapsed if elapsed > 0.0 else 0.0


def split_frame(name, image, head_layout='sbs', head_order='lr'):
    if image.ndim != 3 or image.shape[2] != 3 or image.dtype != np.uint8:
        raise ValueError('Expected an H x W x 3 uint8 BGR image')
    if name != 'head' or head_layout == 'mono':
        return {name: np.ascontiguousarray(image)}
    if head_layout != 'sbs' or head_order not in ('lr', 'rl'):
        raise ValueError('Invalid head_layout or head_order')
    width = image.shape[1]
    if width % 2:
        raise ValueError('Side-by-side head frame must have even width')
    first, second = image[:, :width // 2], image[:, width // 2:]
    left, right = (first, second) if head_order == 'lr' else (second, first)
    return {'head_left': np.ascontiguousarray(left),
            'head_right': np.ascontiguousarray(right)}


def validate_calibration(data, image_size=None):
    width, height = int(data['image_width']), int(data['image_height'])
    if width <= 0 or height <= 0:
        raise ValueError('Calibration image dimensions must be positive')
    if image_size is not None and (width, height) != tuple(image_size):
        raise ValueError(f'Calibration size {(width, height)} != image size {image_size}')
    for key, length, rows, cols in [('camera_matrix', 9, 3, 3),
                                    ('rectification_matrix', 9, 3, 3),
                                    ('projection_matrix', 12, 3, 4)]:
        matrix = data[key]
        values = matrix['data']
        if int(matrix['rows']) != rows or int(matrix['cols']) != cols or len(values) != length:
            raise ValueError(f'Invalid {key} dimensions')
        if not all(math.isfinite(float(x)) for x in values):
            raise ValueError(f'Non-finite {key}')
    k = data['camera_matrix']['data']
    if k[0] <= 0 or k[4] <= 0 or abs(k[8] - 1.0) > 1e-6:
        raise ValueError('Invalid intrinsic matrix')
    distortion = data['distortion_coefficients']
    d = distortion['data']
    expected = {'plumb_bob': (5,), 'rational_polynomial': (8, 12, 14), 'equidistant': (4,)}
    if data['distortion_model'] not in expected or len(d) not in expected[data['distortion_model']]:
        raise ValueError('Unsupported distortion model/coefficient count')
    if int(distortion['rows']) != 1 or int(distortion['cols']) != len(d):
        raise ValueError('Invalid distortion coefficient dimensions')
    if not all(math.isfinite(float(x)) for x in d):
        raise ValueError('Non-finite distortion coefficients')


def store_calibration(directory, name, data, image_size):
    validate_calibration(data, image_size)
    directory = Path(directory).expanduser()
    directory.mkdir(parents=True, exist_ok=True)
    path = directory / (name + '.yaml')
    if path.exists():
        backup = path.with_name(path.name + '.' + str(time.time_ns()) + '.bak')
        backup.write_bytes(path.read_bytes())
    temporary = path.with_suffix('.yaml.tmp')
    temporary.write_text(yaml.safe_dump(data, sort_keys=False), encoding='utf-8')
    temporary.replace(path)
    return path


@dataclass(frozen=True)
class Frame:
    sequence: int
    stamp_ns: int
    received_monotonic: float
    image: np.ndarray


class JpegReceiver:
    """One worker owns every ZMQ socket. Retains one decoded frame per stream."""
    def __init__(self, host, ports=None, stamp_ns=time.time_ns, *, context=None, endpoints=None):
        self.host = host
        self.ports = dict(PORTS if ports is None else ports)
        self.stamp_ns = stamp_ns
        self.context = context
        self.endpoints = endpoints
        self.lock = threading.Lock()
        self.latest = {}
        self.error = None
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self.thread.start()

    def snapshot(self):
        with self.lock:
            return dict(self.latest), self.error

    def close(self):
        self.stop_event.set()
        self.thread.join(timeout=2.0)
        if self.thread.is_alive():
            raise RuntimeError('JPEG receiver did not stop')

    def _run(self):
        context = self.context if self.context is not None else zmq.Context()
        sockets = {}
        counts = {name: 0 for name in self.ports}
        try:
            poller = zmq.Poller()
            for name, port in self.ports.items():
                sock = context.socket(zmq.SUB)
                sockets[sock] = name
                sock.setsockopt(zmq.LINGER, 0)
                sock.setsockopt(zmq.CONFLATE, 1)
                sock.setsockopt(zmq.SUBSCRIBE, b'')
                endpoint = self.endpoints[name] if self.endpoints is not None else f'tcp://{self.host}:{int(port)}'
                sock.connect(endpoint)
                poller.register(sock, zmq.POLLIN)
            while not self.stop_event.is_set():
                for sock, event in poller.poll(50):
                    if not event & zmq.POLLIN:
                        continue
                    raw = sock.recv()
                    stamp = int(self.stamp_ns())
                    received = time.monotonic()
                    image = cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_COLOR)
                    if image is None:
                        continue
                    name = sockets[sock]
                    counts[name] += 1
                    with self.lock:
                        self.latest[name] = Frame(counts[name], stamp, received, image)
        except Exception as exc:
            with self.lock:
                self.error = repr(exc)
        finally:
            for sock in sockets:
                sock.close(0)
            if self.context is None:
                context.term()
