import json
from pathlib import Path
import h5py
import numpy as np
import pyarrow as pa
from astral_data_collect.ownership import PolicyLease
from astral_data_collect.pipeline.intervention import labels_at
from astral_data_collect.pipeline.align_data import align_episode
from astral_data_collect.pipeline.repair_aligned import repair_episode
from astral_data_collect.pipeline.convert_to_lerobot import convert_session
from conftest import write_raw_episode


def test_unknown_and_expired_labels_are_not_expert():
    labels = labels_at({}, [1, 2])
    assert not labels['intervention_valid'].any()
    meta = {'events': [{'name': 'control_owner', 't': 1, 'data': {'activity': 'human'}},
                       {'name': 'control_owner', 't': 2, 'data': {'activity': 'policy', 'takeover_pending': True}}]}
    labels = labels_at(meta, [1.5, 2.5, 5])
    assert labels['intervention'].tolist() == [True, False, False]
    assert labels['intervention_valid'].tolist() == [True, False, False]


def test_lease_expires_and_rejects_reordered_or_invalid_state():
    lease = PolicyLease(timeout=2)
    assert lease.manual_allowed
    assert lease.update(json.dumps({'activity': 'policy', 'epoch': 'a', 'sequence': 2}))
    assert not lease.manual_allowed
    assert not lease.update(json.dumps({'activity': 'human', 'epoch': 'a', 'sequence': 1}))
    assert not lease.manual_allowed
    assert lease.update(json.dumps({'activity': 'human', 'epoch': 'a', 'sequence': 3}))
    assert lease.manual_allowed
    lease.received -= 3
    assert not lease.manual_allowed
    assert not lease.update('null')
    assert not lease.manual_allowed


def test_intervention_survives_alignment_repair_and_export(tmp_path):
    ep = Path(write_raw_episode(str(tmp_path/'src/episode000000')))
    with h5py.File(ep/'robot_data.h5') as f:
        start = f['streams/left_arm_state/timestamps'][0]
    meta_path = ep/'meta.json'
    meta = json.loads(meta_path.read_text())
    meta['events'] = [{'name': 'control_owner', 't': start+i*.5,
                       'data': {'activity': 'human' if i in (1, 2) else 'policy'}} for i in range(10)]
    meta_path.write_text(json.dumps(meta))
    align_episode(str(ep))
    cleaned = tmp_path/'clean/episode000000'
    repair_episode(str(ep), str(cleaned), None, False)
    with h5py.File(ep/'aligned_data.h5') as src, h5py.File(cleaned/'aligned_data.h5') as dst:
        ids = dst['source_frame_index'][:]
        for key in ('control_owner', 'intervention', 'intervention_valid'):
            np.testing.assert_array_equal(dst[key][:], src[key][:][ids])
        assert dst['intervention'][:].any()
    convert_session(str(tmp_path/'clean'), str(tmp_path/'export'))
    import pyarrow.parquet as pq
    table = pq.read_table(next((tmp_path/'export/data').rglob('*.parquet')))
    assert table.schema.field('intervention').type == pa.bool_()
    assert table.schema.field('intervention_valid').type == pa.bool_()
    assert table.schema.field('control_owner').type == pa.int64()
    assert any(table['intervention'].to_pylist())
    info = json.loads((tmp_path/'export/meta/info.json').read_text())
    assert info['features']['intervention']['dtype'] == 'bool'
