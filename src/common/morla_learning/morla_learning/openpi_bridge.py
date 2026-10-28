"""Generic MORLA transforms for OpenPI; no vendor/Aloha joint conventions.

The adapter targets the DataConfigFactory/TrainConfig API and fails explicitly
if a selected locked OpenPI revision does not supply it. See the deployment
guide for the official API references and the unverified external-backend gate.
"""
from __future__ import annotations
import argparse
import dataclasses
import importlib.util
import json
import os
from pathlib import Path
import re

from .contracts import TrainingSpec, LearningContractError, feature_rows


@dataclasses.dataclass(frozen=True)
class ComposedInputs:
    camera_map:dict
    state_dim:int
    action_dim:int

    def __call__(self,data):
        import numpy as np
        state=np.asarray(data['state'],dtype=np.float32)
        if state.shape[-1]!=self.state_dim or not np.isfinite(state).all():
            raise LearningContractError('OpenPI state shape/finite mismatch')
        images=data['images']
        if set(images)!=set(self.camera_map.values()):
            raise LearningContractError('OpenPI images must exactly match frozen camera map')
        result={'state':state,'image':{},'image_mask':{}}
        for slot,key in self.camera_map.items():
            image=np.asarray(images[key])
            if image.ndim!=3: raise LearningContractError('OpenPI image must be rank3')
            if image.shape[0]==3 and image.shape[-1]!=3: image=np.moveaxis(image,0,-1)
            if image.shape[-1]!=3 or image.dtype!=np.uint8:
                raise LearningContractError('OpenPI image must be RGB uint8')
            result['image'][slot]=image; result['image_mask'][slot]=np.True_
        if 'actions' in data:
            actions=np.asarray(data['actions'],dtype=np.float32)
            if actions.shape[-1]!=self.action_dim or not np.isfinite(actions).all():
                raise LearningContractError('OpenPI action shape/finite mismatch')
            result['actions']=actions
        if 'prompt' in data: result['prompt']=data['prompt']
        return result


@dataclasses.dataclass(frozen=True)
class ComposedOutputs:
    action_dim:int
    capacity:int

    def __call__(self,data):
        import numpy as np
        values=np.asarray(data['actions'])
        if values.ndim<1 or values.shape[-1]!=self.capacity or not np.isfinite(values).all():
            raise LearningContractError('OpenPI output capacity/finite mismatch')
        # This is the explicit inverse of declared capacity padding, not a
        # silent reduction of robot degrees of freedom.
        return {'actions':values[...,:self.action_dim].copy()}


def build_config(spec):
    from openpi.training import config as configs
    from openpi import transforms
    base_name=spec.parameters.get('base_config')
    repo_id=spec.parameters.get('repo_id')
    if not isinstance(base_name,str) or not isinstance(repo_id,str) or not re.fullmatch(r'[A-Za-z0-9_-]+/[A-Za-z0-9_-]+',repo_id):
        raise LearningContractError('OpenPI requires explicit base_config and owner/dataset repo_id')
    base=configs.get_config(base_name)
    if int(base.model.action_dim)!=spec.capacity:
        raise LearningContractError('OpenPI model action capacity mismatch')
    if not all(hasattr(configs,n) for n in ('DataConfigFactory','ModelTransformFactory')):
        raise LearningContractError('selected OpenPI API is incompatible with MORLA bridge')
    inputs=ComposedInputs(spec.camera_map,len(feature_rows(spec.features,'state')),len(feature_rows(spec.features,'action')))
    outputs=ComposedOutputs(inputs.action_dim,spec.capacity)
    @dataclasses.dataclass(frozen=True)
    class Factory(configs.DataConfigFactory):
        def create(self,assets_dirs,model_config):
            # Keep standard normalization/model transforms, replace all
            # embodiment-specific transforms with this frozen named contract.
            repack=transforms.RepackTransform({'state':'observation.state','actions':'action',
                                                'images':{key:'observation.images.'+key for key in spec.camera_map.values()}})
            return dataclasses.replace(self.create_base_config(assets_dirs,model_config),
                repack_transforms=transforms.Group(inputs=[repack]),
                data_transforms=transforms.Group(inputs=[inputs],outputs=[outputs]),
                model_transforms=configs.ModelTransformFactory()(model_config),
                action_sequence_keys=('action',),prompt_from_task=True)
    factory=Factory(repo_id=repo_id,assets=dataclasses.replace(base.data.assets,asset_id=spec.dataset_fingerprint))
    parameters=spec.parameters
    permitted={'base_config','repo_id','exp_name','batch_size','num_train_steps','num_workers','wandb_enabled'}
    if set(parameters)-permitted: raise LearningContractError('unsupported OpenPI training overrides')
    overrides={k:v for k,v in parameters.items() if k not in {'base_config','repo_id'}}
    if 'exp_name' not in overrides: raise LearningContractError('OpenPI exp_name required')
    return dataclasses.replace(base,data=factory,checkpoint_base_dir=Path(spec.output_dir),**overrides)


def main(argv=None):
    parser=argparse.ArgumentParser()
    parser.add_argument('spec'); parser.add_argument('--backend-root',required=True)
    parser.add_argument('--stats-only',action='store_true')
    args=parser.parse_args(argv)
    spec=TrainingSpec.from_dict(json.loads(Path(args.spec).read_text()))
    if spec.backend!='openpi': raise LearningContractError('OpenPI spec required')
    root=Path(args.backend_root).resolve()
    from .backends import BackendEnvironment,dataset_contract
    import sys
    BackendEnvironment(sys.executable,str(root),spec.backend_revision).verify(spec)
    dataset_contract(spec)
    # Use a dedicated cache tree so an existing dataset link is never overwritten.
    cache=Path(spec.output_dir)/'morla_contract/datasets'
    target=cache/spec.parameters['repo_id']; target.parent.mkdir(parents=True,exist_ok=True)
    if target.exists() and target.resolve()!=Path(spec.dataset_root).resolve():
        raise LearningContractError('dataset cache link points to another dataset')
    if not target.exists(): target.symlink_to(Path(spec.dataset_root).resolve(),target_is_directory=True)
    os.environ['HF_LEROBOT_HOME']=str(cache)
    config=build_config(spec)
    script=root/'scripts'/('compute_norm_stats.py' if args.stats_only else 'train.py')
    module_spec=importlib.util.spec_from_file_location('morla_external_openpi_entry',script)
    module=importlib.util.module_from_spec(module_spec); module_spec.loader.exec_module(module)
    module.main(config)


if __name__=='__main__': main()
