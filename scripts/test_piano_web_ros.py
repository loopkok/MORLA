#!/usr/bin/env python3
"""End-to-end HTTP -> worker -> real MORLA fake drivers, in ROS domain 74 only."""
import asyncio
import json
import math
import os
from pathlib import Path
import time

import httpx


async def main():
    if os.environ.get("ROS_DOMAIN_ID") != "74" or os.environ.get("ROS_LOCALHOST_ONLY") != "1":
        raise RuntimeError("This software-only integration test requires ROS_DOMAIN_ID=74 ROS_LOCALHOST_ONLY=1")
    from morla_web_manager import web_server as web
    node = web.init_node()
    from std_msgs.msg import String
    from rclpy.qos import qos_profile_sensor_data
    driver_statuses = {}
    status_subscriptions = [node.create_subscription(String, f'/{side}_arm/nero_robot_driver/status',
        lambda message, s=side: driver_statuses.__setitem__(s, json.loads(message.data)), qos_profile_sensor_data)
        for side in ('left','right')]
    pulse = None
    summary = {}
    try:
        async with httpx.AsyncClient(transport=httpx.ASGITransport(app=web.app), base_url="http://testserver", timeout=20) as client:
            async def call(path, body=None, method="POST", expected=200):
                result = await client.request(method, "/api/v1/" + path, json=body)
                assert result.status_code == expected, (path, result.status_code, result.text)
                return result.json().get("data")

            record = await call("piano/config", method="GET")
            cfg = record["config"]
            cfg["timing"].update(prepare_duration_s=1, max_lateness_s=.1, max_arm_velocity_rad_s=.3)
            record = await call("piano/config", {"config": cfg, "revision": record["revision"]})
            scores = await call("piano/scores", method="GET")
            demo = next(row for row in scores if "demo" in row["name"])
            csv = next(row for row in scores if "template" in row["name"])
            opened = await call("piano/session", {"mode": "ros_fake", "revision": record["revision"]})
            token = opened["token"]

            async def heartbeat():
                while True:
                    await call("piano/heartbeat", {"token": token})
                    await asyncio.sleep(.7)

            pulse = asyncio.create_task(heartbeat())

            async def wait(predicate, timeout=12):
                deadline = time.monotonic() + timeout
                while time.monotonic() < deadline:
                    state = await call("piano/state", method="GET")
                    value = predicate(state)
                    if value:
                        return value
                    if state.get("state") == "FAULT":
                        raise AssertionError(state)
                    await asyncio.sleep(.05)
                raise AssertionError(await call("piano/state", method="GET"))

            async def command(operation, timeout=12, **parameters):
                job = await call("piano/command", {"token": token, "operation": operation, **parameters})
                def done(state):
                    result = state.get("result") or {}
                    return result if result.get("job_id") == job["id"] else None
                result = await wait(done, timeout)
                assert result["ok"], result
                return result

            await wait(lambda state: state["state"] == "CONNECTED")
            captured = await command("capture")
            assert set(captured["result"]["positions"]) == {"left_arm", "right_arm", "left_hand", "right_hand"}
            summary["capture"] = True
            await call("infer/cmd", {"cmd": "policy"}, expected=409)
            await call("control/home", {}, expected=409)
            await call("piano/config", {"config": cfg, "revision": record["revision"]}, expected=409)
            await call("piano/heartbeat", {"token": "second-browser"}, expected=409)
            summary["cross_lane_and_owner_rejection"] = True
            async def assert_scope(selected):
                expected = {selected}
                keys = [s+'_'+p for s in ('left','right') for p in ('arm','hand')]
                await wait(lambda state: {key for key in keys if node.count_publishers('/'+key+'/joint_commands')} == expected)
            # Manual calibration operates directly from measured pose, without prepare.
            for side in ("left", "right"):
                await command("joint", side=side, part="arm", joint_index=0, target_deg=.5, calibration_confirmed=True)
                await assert_scope(side+'_arm')
                await wait(lambda state: driver_statuses.get(side, {}).get('enable_status') == 'enabled')
                enable_epoch = driver_statuses[side]['enable_started_monotonic_s']
                await command("joint", side=side, part="arm", joint_index=0, target_deg=1., calibration_confirmed=True)
                retained = await call("piano/state", method="GET")
                seeds = [row for row in retained['events'] if row['event'] == 'manual_control_seed']
                assert seeds[-1]['source'] == 'retained_command'
                assert side+'_arm' in retained['command_targets']
                assert driver_statuses[side]['enable_started_monotonic_s'] == enable_epoch
                summary['manual_'+side+'_continuous_hold_without_reenable'] = True
                await command("joint", side=side, part="hand", joint_index=4, target_deg=8., calibration_confirmed=True)
                await assert_scope(side+'_hand')
                await command("cartesian", side=side, frame="base", axis="z", step=1., calibration_confirmed=True)
                await assert_scope(side+'_arm')
                await command("cartesian", side=side, frame="tool", axis="rz", step=.5, calibration_confirmed=True)
                manual_result = await command("hand_pose", side=side, target_deg=[math.degrees(q) for q in cfg["hands"][side]["hover_rad"]], calibration_confirmed=True)
                await assert_scope(side+'_hand')
            manual = await call("piano/state", method="GET")
            assert manual["state"] == "CALIBRATION_IDLE" and len(manual["poses"]) == 2
            await call("piano/command", {"token": token, "operation": "play", "score_id": demo["id"]}, expected=409)
            summary["manual_joint_cartesian_hand_and_ready_invalidation"] = True
            # Command results contain the post-arrival measured snapshot. The
            # periodically streamed UI snapshot can still precede that result.
            before_positions = manual_result['result']['positions']
            await wait(lambda state: all(side in driver_statuses for side in ('left','right')))
            for side in ('left','right'):
                peer = 'right' if side == 'left' else 'left'
                before_sequences = {s: driver_statuses[s]['position_seq'] for s in ('left','right')}
                result = await command('work_pose', side=side, calibration_confirmed=True, timeout=90)
                positions = result['result']['positions']
                # The peer's preceding hold can finish settling after arrival;
                # prove no new J stage or JS publisher targets the peer.
                assert max(abs(a-b) for a,b in zip(positions[peer+'_arm'],before_positions[peer+'_arm'])) <= cfg['timing']['arrival_tolerance_rad']
                await wait(lambda state: driver_statuses[side]['position_seq'] == before_sequences[side]+4)
                assert driver_statuses[peer]['position_seq'] == before_sequences[peer]
                assert positions['left_hand'] == before_positions['left_hand']
                assert positions['right_hand'] == before_positions['right_hand']
                before_positions = positions
                assert node.count_publishers('/'+side+'_arm/joint_commands') == 1
                assert node.count_publishers('/'+peer+'_arm/joint_commands') == 0
                for hand in ('left','right'):
                    assert node.count_publishers('/'+hand+'_hand/joint_commands') == 0
                summary['selected_'+side+'_work_pose_scoped'] = True
            await command("work_pose", side='both', calibration_confirmed=True, timeout=90)
            working = await call("piano/state", method="GET")
            stages = [event["stage"] for event in working["events"] if event["event"] == "work_pose_stage"]
            assert stages[-5:] == [1, 2, 3, 4, 5], working["events"]
            for side in ("left", "right"):
                expected = [math.radians(q) for q in cfg["calibration"]["work_pose_deg"][side]]
                assert max(abs(a-b) for a, b in zip(working["positions"][side+"_arm"], expected)) < .025
            summary["native_four_stage_J_then_piano_work_pose"] = True
            await command("prepare", timeout=60)
            for side in ("left", "right"):
                for finger in ("index", "middle", "ring", "pinky"):
                    await command("finger", side=side, finger=finger)
                await command("shift", side=side, shift_white=1)
            summary["eight_fingers_and_both_wrists"] = True
            await command("prepare")
            plan = await call("piano/plan", {"score_id": demo["id"], "tempo_bpm": 60})
            assert plan["notes"] == 12
            await command("play", score_id=demo["id"], tempo_bpm=60, timeout=30)
            summary["json_finished"] = True
            await command("prepare")
            job = await call("piano/command", {"token": token, "operation": "play", "score_id": csv["id"], "tempo_bpm": 60})
            await asyncio.sleep(2.3)
            await call("piano/stop", {})
            await wait(lambda state: state["state"] == "STOPPED" and state.get("pressed") is None)
            summary["csv_stop_released_fingers"] = True
            # Accelerate only the idle-cleanup deadline in this software test.
            web._piano_mgr.IDLE_LEASE_SECONDS = web._piano_mgr.LEASE_SECONDS
            pulse.cancel()
            try:
                await pulse
            except asyncio.CancelledError:
                pass
            pulse = None
            await wait(lambda state: not state["active"], timeout=12)
            assert web._piano_mgr.proc is None and web._piano_mgr.drivers is None
            summary["browser_lease_loss_reaped_processes"] = True
            for _ in range(30):
                if not node.robot_command_publishers():
                    break
                await asyncio.sleep(.1)
            assert not node.robot_command_publishers(), node.robot_command_publishers()
            summary["no_motion_publishers_left"] = True
            # Native J mode must be cancelled with stop services, never a JS hold.
            # As in the UI, wait for DDS to remove the crashed worker's writers.
            deadline = time.monotonic() + 35
            while web._piano_blockers("ros_fake"):
                assert time.monotonic() < deadline, web._piano_blockers("ros_fake")
                await asyncio.sleep(.1)
            opened = await call("piano/session", {"mode": "ros_fake", "revision": record["revision"]})
            token = opened["token"]
            pulse = asyncio.create_task(heartbeat())
            await wait(lambda state: state["state"] == "CONNECTED")
            await call("piano/command", {"token": token, "operation": "work_pose", "side": "both", "calibration_confirmed": True})
            await wait(lambda state: state.get("work_progress", {}) and state["work_progress"]["mode"] == "native_j")
            await call("piano/stop", {})
            deadline = time.monotonic() + 8
            while time.monotonic() < deadline:
                interrupted = await call("piano/state", method="GET")
                if interrupted["state"] == "FAULT" and not interrupted["busy"]:
                    break
                await asyncio.sleep(.05)
            else:
                raise AssertionError(interrupted)
            assert not any(event.get("stage") == 5 for event in interrupted["events"] if event["event"] == "work_pose_stage")
            pulse.cancel()
            try:
                await pulse
            except asyncio.CancelledError:
                pass
            pulse = None
            await call("piano/close", {"token": token})
            deadline = time.monotonic() + 12
            while (await call("piano/state", method="GET"))["active"]:
                assert time.monotonic() < deadline, "native J cancelled session did not close"
                await asyncio.sleep(.05)
            summary["native_J_stop_cancels_without_final_pose"] = True
            deadline = time.monotonic() + 12
            while web._piano_blockers("ros_fake"):
                assert time.monotonic() < deadline, web._piano_blockers("ros_fake")
                await asyncio.sleep(.1)
            opened = await call("piano/session", {"mode": "ros_fake", "revision": record["revision"]})
            token = opened["token"]
            pulse = asyncio.create_task(heartbeat())
            await wait(lambda state: state["state"] == "CONNECTED")
            await command("prepare")
            web._piano_mgr.proc.kill()
            await wait(lambda state: not state["active"], timeout=12)
            pulse.cancel()
            try:
                await pulse
            except asyncio.CancelledError:
                pass
            pulse = None
            final = await call("piano/state", method="GET")
            assert final.get("recovery", {}).get("requested") is True, final
            assert final["recovery"].get("exit_code") == 0, final
            assert final["connected"] is False
            summary["worker_crash_stop_only_recovery"] = True
            deadline = time.monotonic() + 20
            while web._piano_blockers("ros_fake"):
                assert time.monotonic() < deadline, web._piano_blockers("ros_fake")
                await asyncio.sleep(.1)
            opened = await call("piano/session", {"mode": "ros_fake", "revision": record["revision"]})
            token = opened["token"]
            pulse = asyncio.create_task(heartbeat())
            await wait(lambda state: state["state"] == "CONNECTED")
            from morla_web_manager.piano_processes import owned_processes, _signal
            import signal
            launch = web._piano_mgr.drivers
            members = owned_processes(launch.piano_scope)
            hands = [(pid,start) for pid,start in members.items()
                     if b"fake_xhands.py" in Path(f"/proc/{pid}/cmdline").read_bytes()]
            assert len(hands) == 1, members
            pulse.cancel()
            try:
                await pulse
            except asyncio.CancelledError:
                pass
            pulse = None
            _signal(*hands[0], signal.SIGKILL)
            await wait(lambda state: not state["active"], timeout=15)
            assert not owned_processes(launch.piano_scope)
            summary["native_driver_exit_closes_and_reaps_lane"] = True
            root = Path(web._piano_store.root)
            (root / "integration-summary.json").write_text(json.dumps(summary, indent=2) + "\n")
            print(json.dumps(summary, indent=2))
    finally:
        if pulse:
            pulse.cancel()
        await asyncio.to_thread(web._piano_mgr.close)
        web.shutdown_node()


if __name__ == "__main__":
    asyncio.run(main())
