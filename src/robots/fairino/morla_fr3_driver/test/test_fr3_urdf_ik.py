"""FR3 asset closure, independent geometry, strict IK and CR5 algorithm parity."""
import hashlib
import json
import math
import xml.etree.ElementTree as ET
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
pytest.importorskip('pinocchio')
from morla_fr3_driver.urdf_ik import IKSolver, MODEL_SHA256, default_urdf_path
from morla_fr3_driver.contract import INITIAL, limits


def manual_fk(q):
    # Independent historical manual reference, deliberately test-only.
    T=np.eye(4)
    for theta,a,d,alpha in zip(q,(0,-.280,-.240,0,0,0),(.140,0,0,.102,.102,.100),
                               (math.pi/2,0,0,math.pi/2,-math.pi/2,0)):
        c,s,ca,sa=math.cos(theta),math.sin(theta),math.cos(alpha),math.sin(alpha)
        T=T@np.array([[c,-s*ca,s*sa,a*c],[s,c*ca,-c*sa,a*s],[0,sa,ca,d],[0,0,0,1]])
    return T


def xml_fk(q):
    T=np.eye(4);index=0
    for joint in ET.parse(default_urdf_path()).getroot().findall('joint'):
        origin=joint.find('origin');O=np.eye(4)
        O[:3,3]=np.fromstring(origin.get('xyz'),sep=' ')
        O[:3,:3]=Rotation.from_euler('xyz',np.fromstring(origin.get('rpy'),sep=' ')).as_matrix()
        A=np.eye(4)
        if joint.get('type')!='fixed':
            A[:3,:3]=Rotation.from_rotvec(np.fromstring(joint.find('axis').get('xyz'),sep=' ')*q[index]).as_matrix()
            index+=1
        T=T@O@A
    return T


def test_only_fr3_assets_hashes_and_independent_multiconfiguration_fk():
    path=default_urdf_path();share=path.parents[1];root=ET.parse(path).getroot()
    provenance=json.loads((share/'PROVENANCE.json').read_text())
    assert hashlib.sha256(path.read_bytes()).hexdigest()==MODEL_SHA256==provenance['target_urdf_sha256']
    assert [j.get('name') for j in root.findall('joint') if j.get('type')!='fixed']==['j'+str(i) for i in range(1,7)]
    assert len(root.findall('link'))==8
    files={p.relative_to(share).as_posix() for p in (share/'meshes').rglob('*') if p.is_file()}
    assert len(files)==7 and files=={m['path'] for m in provenance['meshes']}
    for m in provenance['meshes']:
        assert hashlib.sha256((share/m['path']).read_bytes()).hexdigest()==m['sha256']
    for mesh in root.iter('mesh'):
        uri=mesh.get('filename');assert uri.startswith('package://fr3_description/')
        assert uri.removeprefix('package://fr3_description/') in files
    solver=IKSolver();rng=np.random.default_rng(73)
    for q in [np.zeros(6),np.array(INITIAL),*rng.uniform(-1.4,1.4,(60,6))]:
        actual=solver.fk(q);np.testing.assert_allclose(actual,xml_fk(q),atol=1e-12)
        reference=manual_fk(q)
        assert np.linalg.norm(actual[:3,3]-reference[:3,3])<3e-5
        assert Rotation.from_matrix(reference[:3,:3].T@actual[:3,:3]).magnitude()<3e-5


def test_strict_roundtrips_hash_rejection_limits_and_deadline(tmp_path):
    s=IKSolver(clock=lambda:0);rng=np.random.default_rng(74)
    for _ in range(60):
        seed=np.array(INITIAL)+rng.uniform(-.3,.3,6)
        target=s.fk(seed+rng.uniform(-.02,.02,6));s.sync_state(seed);q=s.solve(target)
        assert q is not None,s.last_report
        assert np.linalg.norm(s.fk(q)[:3,3]-target[:3,3])<=.0002
        assert Rotation.from_matrix(s.fk(q)[:3,:3].T@target[:3,:3]).magnitude()<=.002
        assert np.max(np.abs(q-seed))<=.3
    lo,hi=limits({});np.testing.assert_allclose(s.lower_limits,lo);np.testing.assert_allclose(s.upper_limits,hi)
    assert hi[2]==pytest.approx(math.radians(150)) # Never widen to URDF's 162 degrees.
    bad=tmp_path/'tampered.urdf';bad.write_bytes(default_urdf_path().read_bytes()+b'\n')
    with pytest.raises(ValueError,match='hash'):IKSolver(urdf_path=bad)
    target=s.fk(INITIAL);target[0,3]+=5;s.sync_state(INITIAL)
    assert s.solve(target) is None
    tick=[0]
    def expired():tick[0]+=.1;return tick[0]
    s.clock=expired
    assert s.solve(target) is None and s.last_report['reason']=='deadline'


def test_fr3_and_cr5_algorithms_on_identical_fr3_geometry(monkeypatch):
    # Compare strategies on ONE model; joint values of different robots are not comparable.
    cr5=pytest.importorskip('morla_cr5_driver.urdf_ik')
    from morla_cr5_driver.differential_ik import DifferentialIKSolver as CR5Differential
    from morla_fr3_driver.differential_ik import DifferentialIKSolver
    monkeypatch.setattr(cr5,'MODEL_SHA256',MODEL_SHA256)
    monkeypatch.setattr(cr5,'JOINT_NAMES',tuple('j'+str(i) for i in range(1,7)))
    monkeypatch.setattr(cr5.IKSolver,'tip_frame','flange')
    options=dict(urdf_path=default_urdf_path(),joint_lower=limits({})[0],joint_upper=limits({})[1],clock=lambda:0)
    for fr,reference in [(IKSolver(**options),cr5.IKSolver(**options)),
                         (DifferentialIKSolver(**options),CR5Differential(**options))]:
        q=np.array(INITIAL)
        for i in range(40):
            target=fr.fk(np.array(INITIAL)+.06*np.sin(i/10)*np.arange(1,7)/6)
            fr.sync_state(q);reference.sync_state(q)
            a,b=fr.solve(target),reference.solve(target)
            assert a is not None and b is not None
            np.testing.assert_allclose(a,b,atol=1e-12);q=a
