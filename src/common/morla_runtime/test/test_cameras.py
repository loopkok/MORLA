from types import SimpleNamespace as NS
import pytest
from morla_runtime import FakeAdapter
from morla_runtime.cameras import CameraAdapter, ImageCache


def camera():
    return CameraAdapter({'instance_id': 'cam', 'plugin_id': 'morla.camera.ros_image',
                          'parameters': {}, 'deployment_verified': False},
                         {'device_id': 'serial-camera', 'parameters': {}}, clock=lambda: 1000)


def test_camera_cache_is_latest_only_and_source_clock_fenced():
    reference = FakeAdapter({'ports': {'rgb': {'direction': 'output', 'semantic': 'image'}}}).read(1000)['ports']['rgb']['data']
    cache = ImageCache(clock=lambda: 1000)
    assert cache.accept(reference, source_ns=1)
    assert not cache.accept(reference, source_ns=1)
    assert cache.accept(reference, source_ns=2)
    assert cache.sequence == 2 and cache.dropped == 2
    assert cache.read()['source'] == 'camera' and cache.read()['measured']
    assert not cache.accept(reference, source_ns=1)
    assert cache.fault == 'CAMERA_SOURCE_CLOCK_REGRESSION'
    assert not cache.accept(reference, source_ns=3)
    assert not cache.health(1000)['ready']
    with pytest.raises(ValueError): ImageCache(clock=lambda: 1000).accept(b'bad', source_ns=1)


def test_ros_raw_padded_rgb_image_preserves_dimensions_and_is_measured():
    adapter = camera()
    image = NS(encoding='rgb8', width=2, height=2, step=8,
               data=bytes([255,0,0,0,255,0,99,99, 0,0,255,255,255,255,99,99]),
               header=NS(stamp=NS(sec=10, nanosec=1)))
    adapter._on_image(image)
    sample = adapter.read(1000)['ports']['rgb']
    assert (sample['width'], sample['height']) == (2, 2)
    assert sample['data'].startswith(b'\xff\xd8') and sample['data'].endswith(b'\xff\xd9')
    assert sample['measured'] and not sample['synthetic']
    assert sample['fingerprint']['device_id'] == 'serial-camera'
    assert adapter.health(1000)['ready']
    adapter.close()


def test_ros_unsupported_raw_and_bad_compressed_do_not_become_ready():
    adapter = camera()
    adapter._on_image(NS(encoding='16UC1',width=1,height=1,step=2,data=b'\x00\x01',header=NS(stamp=NS(sec=1,nanosec=1))))
    assert not adapter.health(1000)['ready']
    adapter = camera()
    adapter._on_compressed(NS(format='png',data=b'PNG',header=NS(stamp=NS(sec=1,nanosec=1))))
    assert not adapter.health(1000)['ready']
