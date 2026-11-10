"""Independent bounded SDK camera polling; preserve raw depth units and stamps."""
import copy
import io
import threading
import math
from .transport import stamp_ns


def decode_images(reply, camera, depth_scale=0.):
    import numpy as np
    from PIL import Image
    if camera not in reply.images:raise ValueError('MMK2_CAMERA_MISSING')
    image=reply.images[camera];stamp=stamp_ns(image.stamp)
    if stamp<=0:raise ValueError('MMK2_CAMERA_TIMESTAMP_MISSING')
    ports={}
    for source,name in [('color','rgb'),('depth','depth')]:
        if source not in image.data:continue
        raw=image.data[source];shape=list(raw.shape)
        dtype=np.dtype(raw.dtype)
        if dtype not in (np.dtype('uint8'),np.dtype('uint16')):raise ValueError('MMK2_CAMERA_DTYPE')
        if not shape or any(n<=0 for n in shape) or math.prod(shape)*dtype.itemsize!=len(raw.data) or len(raw.data)>24*1024*1024:
            raise ValueError('MMK2_CAMERA_SHAPE')
        array=np.frombuffer(raw.data,dtype=dtype).reshape(shape)
        if len(shape)==1:
            decoded=Image.open(io.BytesIO(raw.data))
            if decoded.width*decoded.height>8_000_000:raise ValueError('MMK2_CAMERA_DIMENSIONS')
            array=np.asarray(decoded.convert('RGB') if name=='rgb' else decoded)
        elif name=='rgb':
            # SDK raw/OpenCV camera frames use BGR. JPEG decode above is RGB.
            if array.ndim!=3 or array.shape[2]!=3 or dtype!=np.dtype('uint8'):raise ValueError('MMK2_RGB_SHAPE')
            array=array[:,:,::-1]
        if name=='rgb':
            out=io.BytesIO();Image.fromarray(array).save(out,format='JPEG')
            ports[name]=dict(data=out.getvalue(),encoding='jpeg',width=array.shape[1],height=array.shape[0],unit='rgb')
        else:
            if array.ndim!=2 or array.dtype!=np.dtype('uint16'):raise ValueError('MMK2_DEPTH_SHAPE')
            out=io.BytesIO();Image.fromarray(array).save(out,format='PNG')
            ports[name]=dict(data=out.getvalue(),encoding='png',width=array.shape[1],height=array.shape[0],unit='sdk_depth',depth_scale_m=depth_scale or None)
    if not ports:raise ValueError('MMK2_CAMERA_EMPTY')
    return stamp,ports


class CameraAdapter:
    device_scoped_stop=False
    def __init__(self, instance, session):
        self.instance=instance;self.session=session;self.p=instance['parameters'];self.shutdown=threading.Event();self.thread=None;self.lock=threading.Lock();self.sample={'ports':{},'sample_ns':0,'ready':False};self.previous=0;session.captures.append(self)
    def configure(self, context):
        self.session.start()
        self.thread=threading.Thread(target=self._run,name='mmk2-camera',daemon=True);self.thread.start()
    def _run(self):
        while not self.shutdown.is_set():
            try:
                stamp,ports=decode_images(self.session.transport.images(self.p['camera']),self.p['camera'],self.p['depth_scale_m'])
                if stamp<self.previous:raise ValueError('MMK2_CAMERA_CLOCK_REGRESSED')
                if stamp>self.previous:
                    now=self.session.clock();self.previous=stamp
                    for port in ports.values():port.update(sample_ns=now,source_ns=stamp,valid=True,source='hardware' if self.session.backend=='real' else 'fake',measured=self.session.backend=='real',synthetic=self.session.backend=='fake')
                    with self.lock:self.sample=dict(ports=ports,sample_ns=now,source_ns=stamp,ready=True)
            except Exception as exc:
                with self.lock:self.sample.update(ready=False,reason=str(exc))
            self.shutdown.wait(1/self.p['fps'])
    def read(self, now_ns):
        with self.lock:sample=copy.deepcopy(self.sample)
        sample['ready']=sample['ready'] and not self.session.closed and 0<=self.session.clock()-sample['sample_ns']<=self.p['feedback_timeout_s']*1e9
        sample.update(source='hardware' if self.session.backend=='real' else 'fake',measured=self.session.backend=='real' and bool(sample['sample_ns']),synthetic=self.session.backend=='fake')
        for port in sample['ports'].values():port['valid']=sample['ready']
        return sample
    def health(self, now_ns):
        sample=self.read(now_ns)
        for port in sample['ports'].values():port.pop('data',None)
        return sample
    def stop(self):return True # sensor stop is not a robot motor stop
    def release(self):return True
    def close(self):self.session.close()
    def _close_capture(self):
        self.shutdown.set()
        if self.thread:
            self.thread.join(timeout=self.p['rpc_timeout_s']+1)
            if self.thread.is_alive():raise TimeoutError('MMK2_CAMERA_WORKER_DID_NOT_EXIT')
