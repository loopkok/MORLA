import json
from pathlib import Path
import pytest
from morla_teleop.gripper_processing import PinchFilter,TriggerFilter,FilteredGripperController


def test_exact_legacy_numerical_trace():
    # Captured from the pre-extraction ROS node via AST; source SHA is frozen in fixture.
    trace=json.loads((Path(__file__).parent/'fixtures/legacy_gripper_trace.json').read_text())['trace']
    pinch,trigger=PinchFilter(),TriggerFilter()
    for row in trace:
        assert pinch.step(row['distance'],row['time'])==pytest.approx(row['pinch'],abs=1e-14)
        assert [pinch.low,pinch.high]==pytest.approx(row['envelope'],abs=1e-14)
        assert trigger.step(row['trigger'])==pytest.approx(row['trigger_filtered'],abs=1e-14)


def test_filter_reset_does_not_reuse_old_device_range():
    pinch=PinchFilter();pinch.step(.05,1);pinch.step(.08,2);pinch.reset()
    assert pinch.low is None and pinch.ratio is None
    trigger=TriggerFilter();trigger.step(1);trigger.reset()
    assert trigger.step(0)==0


def test_same_filtered_trigger_controls_metric_or_ratio_target():
    for out,expected in [([.05,0],0),([0,1],1)]:
        params={'output_range':out}
        controller=FilteredGripperController(params,clock=lambda:100)
        assert controller(1,[0],0,params)==pytest.approx([expected])


def test_legacy_slew_shaping_initial_snap_and_dt_cap():
    from morla_teleop.gripper_processing import limit_ratio_step
    assert limit_ratio_step(.8,0,0,2.5)==.8
    assert limit_ratio_step(1,.5,.02,2.5)==pytest.approx(.55)
    assert limit_ratio_step(0,.5,1,2.5)==pytest.approx(.25)
    assert limit_ratio_step(1,.5,.02,0)==1
