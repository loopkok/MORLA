"""Numerical and end-to-end checks using the actual migrated CR5 model."""
import hashlib
import json
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
pin=pytest.importorskip('pinocchio')
from morla_cr5_driver.urdf_ik import IKSolver,MODEL_SHA256,default_urdf_path


def xml_fk(q):
    """Independent URDF transform product; does not call Pinocchio FK."""
    root=ET.parse(default_urdf_path()).getroot();T=np.eye(4)
    for joint,angle in zip(root.findall('joint'),q):
        origin=joint.find('origin');O=np.eye(4)
        O[:3,3]=np.fromstring(origin.get('xyz'),sep=' ')
        O[:3,:3]=Rotation.from_euler('xyz',np.fromstring(origin.get('rpy'),sep=' ')).as_matrix()
        A=np.eye(4);A[:3,:3]=Rotation.from_rotvec(np.fromstring(joint.find('axis').get('xyz'),sep=' ')*angle).as_matrix()
        T=T@O@A
    return T


def test_arm_only_model_mesh_closure_and_fk():
    path=default_urdf_path();root=ET.parse(path).getroot();share=path.parents[2]
    assert hashlib.sha256(path.read_bytes()).hexdigest()==MODEL_SHA256
    assert [j.get('name') for j in root.findall('joint')]==['joint'+str(i) for i in range(1,7)]
    assert len(root.findall('link'))==7
    assert not root.findall('ros2_control')
    for mesh in root.iter('mesh'):
        assert mesh.get('filename').startswith('package://robot_description/meshes/cr5/')
        assert (share/mesh.get('filename').split('package://robot_description/')[1]).is_file()
    s=IKSolver();rng=np.random.default_rng(41)
    for q in rng.uniform(-2,2,(30,6)):
        np.testing.assert_allclose(s.fk(q),xml_fk(q),atol=1e-12)
    np.testing.assert_allclose(s.fk(np.zeros(6))[:3,3],[.00000282837076,-.246,1.04700039],atol=1e-8)


def test_local_roundtrips_and_continuity():
    s=IKSolver();rng=np.random.default_rng(42)
    for _ in range(100):
        seed=rng.uniform(-1.5,1.5,6)
        target=s.fk(seed+rng.uniform(-.03,.03,6));s.sync_state(seed)
        q=s.solve(target)
        assert q is not None,s.last_report
        np.testing.assert_allclose(s.fk(q)[:3,3],target[:3,3],atol=.0002)
        assert Rotation.from_matrix(s.fk(q)[:3,:3].T@target[:3,:3]).magnitude()<=.002
        assert np.max(np.abs(q-seed))<=.3
    # Track a continuous reachable path without wrapping to equivalent far joints.
    start=np.array([.3,-.7,.9,-.4,.6,.2]);q=start.copy()
    for t in np.linspace(0,1,60):
        target=s.fk(start+.04*np.sin(t)*np.arange(1,7)/6)
        s.sync_state(q);n=s.solve(target)
        assert n is not None and np.max(np.abs(n-q))<.03
        q=n


def test_joint_mapping_and_intersected_limits():
    signs=np.array([-1,1,-1,1,-1,1]);offsets=np.array([.1,-.1,.1,0,.2,0])
    s=IKSolver(joint_signs=signs,joint_offsets=offsets,joint_lower=[-1]*6,joint_upper=[1]*6)
    q=np.array([.2,-.4,.5,.3,-.5,.1]);np.testing.assert_allclose(s.fk(q),xml_fk(signs*q+offsets),atol=1e-12)
    s.sync_state(q);target=s.fk(q+.01);result=s.solve(target)
    assert result is not None
    with pytest.raises(ValueError,match='outside'):s.sync_state([2]*6)
    with pytest.raises(ValueError,match='intersection'):IKSolver(joint_lower=[10]*6,joint_upper=[11]*6)
    with pytest.raises(ValueError,match='signs'):IKSolver(joint_signs=[0]*6)
    # joint3 URDF limits are narrower than the old driver outer guard.
    assert IKSolver().upper_limits[2]==2.7925


def test_unreachable_singular_deadline_and_bad_model(tmp_path):
    s=IKSolver();s.sync_state(np.zeros(6));target=s.fk(np.zeros(6));target[0,3]+=5
    assert s.solve(target) is None
    assert np.isfinite(s.seed).all()
    # At the singular zero posture, holding still succeeds without numerical noise.
    np.testing.assert_allclose(s.solve(s.fk(np.zeros(6))),np.zeros(6))
    ticks=iter([0,1,1]);s.clock=lambda:next(ticks);assert s.solve(target) is None
    assert s.last_report['reason']=='deadline'
    path=tmp_path/'bad.urdf';path.write_text('<robot name="bad"/>')
    with pytest.raises(ValueError,match='hash'):IKSolver(urdf_path=path)


def test_installed_asset_resolution_and_geometry_contract(tmp_path,monkeypatch):
    from morla_core import resource_locator
    share=tmp_path/'robot_description';(share/'config').mkdir(parents=True)
    source=default_urdf_path().parents[2]
    shutil.copytree(source/'config',share/'config',dirs_exist_ok=True)
    shutil.copytree(source/'urdf/cr5',share/'urdf/cr5')
    monkeypatch.setattr(resource_locator,'package_share',lambda name:share)
    assert default_urdf_path()==share/'urdf/cr5/cr5.urdf'
    assert IKSolver().model_sha256==MODEL_SHA256
    d=json.loads((Path(__file__).parents[1]/'descriptor.json').read_text())
    assert d['geometry']['model_sha256']==MODEL_SHA256


def test_real_factory_and_probe_never_call_controller_ik():
    from morla_cr5_driver.vr_controller import create_controller
    from morla_cr5_driver.kinematics_check import probe
    def forbidden():raise AssertionError('Controller IK provider must not be used')
    c=create_controller({'instance':{'plugin_id':'morla.dobot.cr5','backend':'real',
        'parameters':{'kinematics_verified':True,'joint_lower':[-2]*6,'joint_upper':[2]*6}},
        'binding':{'parameters':{'calibration_verified':True,'input_rotation':np.eye(3).tolist(),
            'solver_tip_from_tool':np.eye(4).tolist()}},'kinematics_provider':forbidden})
    try:
        assert c.requires_fresh_input
        assert isinstance(c.controller.solver,IKSolver)
    finally:c.close()
    class ReadOnly:
        def read(self):return {'values':[.2,-.5,.7,.3,-.4,.1]}
        def inverse_kinematics(self,*args):forbidden()
        def forward_kinematics(self,*args):forbidden()
    report=probe(ReadOnly())
    assert report['passed'] and not report['physical_base_tcp_verified']
    assert report['solver']=='cr5_urdf_pinocchio_dls'
