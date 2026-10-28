"""Real camera providers with independent capture and a bounded latest sample.

These are acquisition adapters, not preview servers. No renderer/browser is in
this path; a slow consumer only drops old cached images. Optional camera imports
happen exclusively during explicit configure of a selected real camera.
"""
import copy
import io
import multiprocessing
import os
from pathlib import Path
import queue
import threading
import time


class ImageCache:
    def __init__(self, clock=time.monotonic_ns, fingerprint=None):
        self.clock, self.fingerprint = clock, copy.deepcopy(fingerprint or {})
        self.lock = threading.Lock()
        self.sample = None
        self.sequence = 0
        self.dropped = 0
        self.last_source_ns = None
        self.fault = None

    def accept(self, jpeg, *, source_ns, width=None, height=None, received_ns=None):
        if not isinstance(jpeg, (bytes, bytearray)) or not jpeg.startswith(b'\xff\xd8') or not jpeg.endswith(b'\xff\xd9'):
            raise ValueError('CAMERA_INVALID_JPEG')
        now = self.clock()
        received_ns = now if received_ns is None else received_ns
        if type(source_ns) is not int or source_ns <= 0 or type(received_ns) is not int or received_ns > now:
            raise ValueError('CAMERA_INVALID_TIME')
        with self.lock:
            if self.fault:
                return False
            if self.last_source_ns is not None:
                if source_ns < self.last_source_ns:
                    self.fault = 'CAMERA_SOURCE_CLOCK_REGRESSION'
                    self.sample = None
                    return False
                if source_ns == self.last_source_ns:
                    self.dropped += 1
                    return False
            if self.sample is not None:
                self.dropped += 1
            self.last_source_ns = source_ns
            self.sequence += 1
            self.sample = {'data': bytes(jpeg), 'encoding': 'jpeg', 'sample_ns': received_ns,
                           'source_ns': source_ns, 'sequence': self.sequence, 'source': 'camera',
                           'synthetic': False, 'measured': True, 'valid': True,
                           'width': width, 'height': height, 'fingerprint': self.fingerprint}
            return True

    def read(self):
        with self.lock:
            return copy.deepcopy(self.sample)

    def health(self, now_ns):
        with self.lock:
            sample = self.sample
            return {'ready': sample is not None and self.fault is None,
                    'sample_ns': sample['sample_ns'] if sample else 0,
                    'source': 'camera', 'synthetic': False, 'measured': sample is not None,
                    'reason': self.fault or ('WAITING_CAMERA_FRAME' if sample is None else ''),
                    'sequence': self.sequence, 'dropped': self.dropped,
                    'fingerprint': copy.deepcopy(self.fingerprint), 'ports': {}}


def _uvc_worker(path, settings, output):
    import cv2
    capture = cv2.VideoCapture(path)
    try:
        if not capture.isOpened():
            output.put({'error': 'UVC_OPEN_FAILED'})
            return
        for key, cv_key in (('width', cv2.CAP_PROP_FRAME_WIDTH), ('height', cv2.CAP_PROP_FRAME_HEIGHT), ('fps', cv2.CAP_PROP_FPS)):
            if settings.get(key):
                capture.set(cv_key, settings[key])
        while True:
            ok, frame = capture.read()
            stamp = time.monotonic_ns()
            if not ok:
                raise RuntimeError('UVC_READ_FAILED')
            encoded, data = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 90])
            if not encoded:
                raise RuntimeError('UVC_ENCODE_FAILED')
            item = {'jpeg': data.tobytes(), 'source_ns': stamp, 'received_ns': stamp,
                    'width': frame.shape[1], 'height': frame.shape[0]}
            try:
                output.put_nowait(item)
            except queue.Full:
                try:
                    output.get_nowait()
                except queue.Empty:
                    pass
                try:
                    output.put_nowait(item)
                except queue.Full:
                    pass
    except Exception as exc:
        try:
            output.put_nowait({'error': str(exc)})
        except queue.Full:
            pass
    finally:
        capture.release()


class CameraAdapter:
    backend = 'real'
    watchdog_verified = False  # Camera has no motor activation operation.
    def __init__(self, instance, device, clock=time.monotonic_ns):
        self.instance, self.device, self.clock = copy.deepcopy(instance), copy.deepcopy(device), clock
        self.settings = {**instance.get('parameters', {}), **device.get('parameters', {})}
        self.deployment_verified = bool(instance.get('deployment_verified'))
        self.cache = ImageCache(clock, {'device_id': device.get('device_id'), 'verified': self.deployment_verified,
                                        'source': 'deployment_binding'})
        self.context = None
        self.closed = False
        self.process, self.thread = None, None
        self.stop_event = threading.Event()

    def configure(self, context):
        self.context = context
        plugin = self.instance['plugin_id']
        kind = self.settings.get('camera_backend')
        if plugin == 'morla.camera.uvc': kind = 'uvc'
        if plugin == 'morla.camera.ros_image': kind = 'ros_image'
        if kind == 'uvc':
            self._configure_uvc()
        elif kind == 'ros_image':
            self._configure_ros()
        else:
            raise ValueError('CAMERA_BACKEND_REQUIRED')

    def _configure_uvc(self):
        import importlib.util
        if importlib.util.find_spec('cv2') is None:
            raise RuntimeError('UVC_BACKEND_REQUIRES_OPENCV')
        path = Path(self.settings.get('device_path', ''))
        if not str(path).startswith('/dev/v4l/by-id/') or not path.exists():
            raise ValueError('UVC_REQUIRES_STABLE_BY_ID_PATH')
        if self.device.get('device_id') != path.name:
            raise ValueError('UVC_FINGERPRINT_MISMATCH')
        identity = path.stat()
        self.cache.fingerprint.update(path=str(path), resolved_path=str(path.resolve()),
                                      stat_device=identity.st_rdev, source='stable_device_path')
        ctx = multiprocessing.get_context('spawn')
        self.output = ctx.Queue(maxsize=1)
        self.process = ctx.Process(target=_uvc_worker, args=(str(path), self.settings, self.output), daemon=True)
        self.process.start()
        self.thread = threading.Thread(target=self._pump_uvc, daemon=True)
        self.thread.start()

    def _pump_uvc(self):
        while not self.stop_event.is_set():
            try:
                item = self.output.get(timeout=.05)
            except queue.Empty:
                if not self.process.is_alive():
                    self.cache.fault = 'CAMERA_WORKER_EXITED'
                    break
                continue
            if 'error' in item:
                self.cache.fault = item['error']
                break
            try:
                self.cache.accept(item.pop('jpeg'), **item)
            except ValueError as exc:
                self.cache.fault = str(exc)
                break

    def _configure_ros(self):
        import rclpy
        from rclpy.node import Node
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Image, CompressedImage
        topic = self.settings.get('image_topic')
        if not isinstance(topic, str) or not topic.startswith('/') or '//' in topic or '..' in topic:
            raise ValueError('CAMERA_IMAGE_TOPIC_REQUIRED')
        if not rclpy.ok(): rclpy.init()
        self.node = Node('camera_source_' + self.instance['instance_id'])
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        compressed = self.settings.get('compressed', False)
        self.node.create_subscription(CompressedImage if compressed else Image, topic,
                                      self._on_compressed if compressed else self._on_image, qos_profile_sensor_data)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()

    @staticmethod
    def _source_ns(message):
        stamp = message.header.stamp
        if not 0 <= stamp.nanosec < 1_000_000_000:
            raise ValueError('CAMERA_INVALID_SOURCE_STAMP')
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    def _on_compressed(self, message):
        try:
            if 'jpeg' not in message.format.lower() and 'jpg' not in message.format.lower():
                raise ValueError('CAMERA_COMPRESSED_ENCODING_UNSUPPORTED')
            self.cache.accept(bytes(message.data), source_ns=self._source_ns(message))
        except ValueError as exc:
            self.cache.fault = str(exc)

    def _on_image(self, message):
        try:
            import numpy as np
            from PIL import Image
            encoding = message.encoding.lower()
            channels = {'rgb8': 3, 'bgr8': 3, 'mono8': 1}.get(encoding)
            if channels is None or message.width <= 0 or message.height <= 0 or message.step < message.width * channels:
                raise ValueError('CAMERA_RAW_ENCODING_OR_SHAPE_UNSUPPORTED')
            raw = np.frombuffer(bytes(message.data), dtype=np.uint8)
            if raw.size != message.height * message.step:
                raise ValueError('CAMERA_RAW_BUFFER_SHAPE')
            pixels = raw.reshape(message.height, message.step)[:, :message.width * channels]
            pixels = pixels.reshape(message.height, message.width, channels)
            if encoding == 'bgr8': pixels = pixels[..., ::-1]
            if channels == 1: pixels = pixels[..., 0]
            output = io.BytesIO()
            Image.fromarray(pixels).save(output, format='JPEG', quality=90)
            self.cache.accept(output.getvalue(), source_ns=self._source_ns(message), width=message.width, height=message.height)
        except (ValueError, ImportError) as exc:
            self.cache.fault = str(exc)

    def health(self, now_ns):
        return self.cache.health(now_ns)

    def read(self, now_ns):
        sample = self.cache.read()
        return {'sample_ns': sample['sample_ns'] if sample else 0, 'source': 'camera', 'synthetic': False,
                'measured': sample is not None, 'ports': {'rgb': sample} if sample else {}}

    def close(self):
        if self.closed: return
        self.closed = True
        self.stop_event.set()
        if self.process is not None:
            self.process.terminate()
            self.process.join(timeout=.5)
            if self.process.is_alive():
                self.process.kill()
                self.process.join(timeout=.5)
            self.output.close()
        if hasattr(self, 'executor'):
            self.executor.shutdown(timeout_sec=.5)
        if self.thread is not None:
            self.thread.join(timeout=.5)
        if hasattr(self, 'node'): self.node.destroy_node()


def make_camera_adapters(runtime):
    result = {}
    for device in runtime.resolved.get('device_sessions', []):
        if device.get('backend') != 'real': continue
        for key in device['instances']:
            instance = runtime.instances[key]
            if instance['plugin_id'] in ('morla.camera', 'morla.camera.ros_image', 'morla.camera.uvc'):
                result[key] = CameraAdapter(instance, device, runtime.clock)
    return result
