import importlib.util
from pathlib import Path
import numpy as np
import pytest
mujoco=pytest.importorskip('mujoco')
from morla_core.composition import compile_system

ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('scene',ROOT/'scripts/composable_scene.py')
scene=importlib.util.module_from_spec(spec);spec.loader.exec_module(scene)

def assembly(instances,mounts=None):
    return compile_system({'assembly_version':1,'assembly_id':'scene_test','revision':1,'instances':instances,'mounts':mounts or []})

def test_mixed_scene_and_named_indices():
    result=scene.build_scene(assembly({'a':{'plugin':'morla.nero.arm'},'b':{'plugin':'morla.astral.arm','parameters':{'side':'right'}},'hand':{'plugin':'morla.wuji.hand'}},[{'child':'hand.mount','parent':'a.flange','transform':[0,0,0,0,0,0,1]}]))
    model=mujoco.MjModel.from_xml_string(result['xml']);data=mujoco.MjData(model)
    assert model.nq==model.nu==34
    assert len(result['joint_map']['a'])==len(result['joint_map']['b'])==7
    assert model.body('hand__mount').parentid==model.body('a__left_link7').id
    for _ in range(100):mujoco.mj_step(model,data)
    assert np.isfinite(data.qpos).all()
    assert result['simulation_level']=='legacy_asset_dynamics_collision_unverified'

@pytest.mark.parametrize('plugin,side,asset',[('morla.nero.arm','left','xnero/xnero_dual.xml'),('morla.astral.arm','right','astral/astral_dual_clean.xml')])
def test_zero_pose_fk_preserves_legacy_asset(plugin,side,asset):
    original=mujoco.MjModel.from_xml_path(str(ROOT/'mjcf'/asset));before=mujoco.MjData(original);mujoco.mj_forward(original,before)
    composed=scene.build_scene(assembly({'one':{'plugin':plugin,'parameters':{'side':side}}}))
    model=mujoco.MjModel.from_xml_string(composed['xml']);after=mujoco.MjData(model);mujoco.mj_forward(model,after)
    np.testing.assert_allclose(after.xpos[model.body('one__'+side+'_link7').id],before.xpos[original.body(side+'_link7').id],atol=1e-10)
    np.testing.assert_allclose(after.xmat[model.body('one__'+side+'_link7').id],before.xmat[original.body(side+'_link7').id],atol=1e-10)

def test_repeated_same_source_prefixes():
    result=scene.build_scene(assembly({i:{'plugin':'morla.nero.arm'} for i in ('first','second','third')}))
    model=mujoco.MjModel.from_xml_string(result['xml'])
    assert model.nq==21
    assert len({model.joint(i).name for i in range(model.njnt)})==21

def test_missing_calibration_never_identity():
    value=assembly({'a':{'plugin':'morla.nero.arm'},'b':{'plugin':'morla.wuji.hand'}},[{'child':'b.mount','parent':'a.flange','calibration':'unresolved'}])
    with pytest.raises(ValueError,match='resolved transform'):scene.build_scene(value)

def test_nonroot_mount_child_frame_explicitly_rejected():
    value=assembly({'a':{'plugin':'morla.nero.arm'},'b':{'plugin':'morla.wuji.hand'}},[{'child':'b.flange','parent':'a.flange','transform':[0,0,0,0,0,0,1]}])
    with pytest.raises(ValueError,match='child endpoint'):scene.build_scene(value)

def test_canonical_joint_order_does_not_guess_source_order():
    value=assembly({'a':{'plugin':'morla.nero.arm'}})
    value['instances'][0]['joints'].reverse()
    result=scene.build_scene(value)
    assert result['joint_map']['a']['joint1']=='a__left_joint1'
    value['instances'][0]['joints'][0]['mjcf_name']='not_a_joint'
    with pytest.raises(ValueError,match='explicit MJCF joint'):scene.build_scene(value)
