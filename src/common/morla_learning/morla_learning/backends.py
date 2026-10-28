"""Learning backend adapters. External algorithms run in their locked environments."""
from __future__ import annotations
from dataclasses import dataclass
import hashlib
import json
from pathlib import Path
import re
import subprocess

from .contracts import TrainingSpec, LearningContractError, feature_rows, camera_keys


def file_hash(path):
    value=hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda:stream.read(1024*1024),b''): value.update(block)
    return value.hexdigest()


def checkpoint_hash(root):
    """Hash content and relative paths; exclude the manifest that records this hash."""
    root=Path(root).resolve()
    if root.is_file(): return file_hash(root)
    if not root.is_dir(): raise LearningContractError('checkpoint not found')
    digest=hashlib.sha256(); count=0
    for path in sorted(root.rglob('*')):
        if not path.is_file() or path.name=='model_manifest.json': continue
        if not path.resolve().is_relative_to(root): raise LearningContractError('checkpoint symlink escapes root')
        relative=str(path.relative_to(root)).encode()
        digest.update(len(relative).to_bytes(8,'big'));digest.update(relative)
        digest.update(bytes.fromhex(file_hash(path)));count+=1
    if not count: raise LearningContractError('checkpoint has no model files')
    return digest.hexdigest()


def dataset_contract(spec:TrainingSpec):
    """Check the actual exported dataset, independently of the spec's dimension claim."""
    root=Path(spec.dataset_root)
    info=json.loads((root/'meta/info.json').read_text())
    features=info.get('features',{})
    for key,kind in (('observation.state','state'),('action','action')):
        rows=feature_rows(spec.features,kind)
        expected=[r if isinstance(r,str) else r.get('name',r.get('feature_id')) for r in rows]
        actual=features.get(key,{})
        if actual.get('shape')!=[len(expected)] or actual.get('names')!=expected:
            raise LearningContractError(f'dataset {key} names/shape mismatch')
    for camera in camera_keys(spec.features):
        if 'observation.images.'+camera not in features:
            raise LearningContractError('dataset missing camera '+camera)
    fps=spec.features.get('dataset_fps')
    if fps is not None and info.get('fps')!=fps:
        raise LearningContractError('dataset fps mismatch')
    return info


@dataclass(frozen=True)
class BackendEnvironment:
    python: str
    root: str
    revision: str

    def verify(self,spec):
        root=Path(self.root).expanduser().resolve()
        if not Path(self.python).is_file() or not root.is_dir():
            raise LearningContractError('backend Python/source not installed')
        if spec.backend_revision!=self.revision:
            raise LearningContractError('backend revision does not match frozen spec')
        actual=subprocess.run(['git','rev-parse','HEAD'],cwd=root,text=True,capture_output=True,check=True).stdout.strip()
        if actual!=self.revision:
            raise LearningContractError('backend source commit mismatch')
        dirty=subprocess.run(['git','diff','--quiet','HEAD'],cwd=root).returncode
        if dirty:
            raise LearningContractError('backend source has uncommitted changes; freeze its patch first')
        return root


def training_command(spec,environment):
    root=environment.verify(spec)
    dataset_contract(spec)
    path=spec.write_config(Path(spec.output_dir)/'morla_contract')
    if spec.backend=='act':
        executable=next((p for p in [Path(environment.python).with_name('lerobot-train')]
                         if p.is_file()),None)
        if executable is None:
            raise LearningContractError('lerobot-train is not installed in selected environment')
        return spec.argv(str(executable)),root
    return [environment.python,'-m','morla_learning.openpi_bridge',str(path),'--backend-root',str(root)],root


def run_training(spec,environment,*,dry_run=True):
    argv,root=training_command(spec,environment)
    if dry_run: return {'argv':argv,'cwd':str(root),'status':'config_valid'}
    subprocess.run(argv,cwd=root,check=True)
    return {'status':'training_process_completed','contract':str(Path(spec.output_dir)/'morla_contract/training_spec.json')}
