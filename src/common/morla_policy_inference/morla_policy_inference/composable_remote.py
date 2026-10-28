"""Manifest-authenticated predictor on the established MORLA websocket protocol."""
import numpy as np
from morla_learning import ModelManifest
from morla_learning.contracts import feature_rows,LearningContractError
from .client import WebsocketClient


class ManifestRemotePredictor:
    def __init__(self,manifest,*,host='127.0.0.1',port=8001,prompt='',timeout_s=.2,client_factory=WebsocketClient):
        self.manifest,self.prompt=manifest,prompt
        self.client=client_factory(host=host,port=port,connect_timeout_s=2,infer_timeout_s=timeout_s)
        try:
            raw=self.client.get_server_metadata().get('model_manifest')
            if not raw or ModelManifest.from_dict(raw).to_dict()!=manifest.to_dict():
                raise LearningContractError('policy server actual model manifest differs from selected checkpoint')
        except Exception:
            self.client.close();raise

    def __call__(self,observation):
        request={'observation/state':observation['state'],'prompt':self.prompt}
        for slot,label in self.manifest.camera_map.items():
            if label not in observation['images']: raise LearningContractError('missing model camera '+label)
            request['observation/camera/'+slot]=observation['images'][label]
        response=self.client.infer(request)
        values=np.asarray(response.get('actions'),dtype=np.float64)
        dim=len(feature_rows(self.manifest.features,'action'))
        if values.ndim==1: values=values[None,:]
        if values.ndim!=2 or not len(values) or values.shape[1]!=dim or not np.isfinite(values).all():
            raise LearningContractError('remote policy action contract mismatch')
        return values

    def close(self):self.client.close()
