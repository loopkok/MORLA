import time
import numpy as np
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, NeroFW

def wait_for_motion(robot, timeout=25.0):
    """等待运动完成的辅助函数"""
    start_time = time.time()
    time.sleep(0.5) # 给底层缓冲时间开始运动
    while True:
        status = robot.get_arm_status()
        # motion_status == 0 表示到达指定点位
        if status is not None and status.msg.motion_status == 0:
            return True
        if time.time() - start_time > timeout:
            print("   [警告] 等待运动完成超时，可能受到了限位或阻力。")
            return False
        time.sleep(0.1)

def main():
    # ================= 1. 配置与连接 =================
    cfg = create_agx_arm_config(
        robot=ArmModel.NERO, 
        firmeware_version=NeroFW.DEFAULT,  # 使用你上次代码里的 DEFAULT 固件版本
        channel="can_nero_right"           # 使用你上次代码里的通道名称
    )
    
    robot = AgxArmFactory.create_arm(cfg)
    end_effector = robot.init_effector(robot.OPTIONS.EFFECTOR.AGX_GRIPPER)
    robot.connect()
    
    print("⏳ 正在使能机械臂...")
    while not robot.enable():
        time.sleep(0.1)
        
    print("✅ 使能成功！")
    
    # ================= 2. 强行进入绝对安全的数学姿态 =================
    # 先设置稍快一点的速度移动关节，避免等太久
    robot.set_speed_percent(5)
    
    print("\n👉 正在将机械臂移动到安全的微曲测试姿态 (避开所有奇异点和死角)...")
    print("   ⚠️ 警告：请确保机械臂伸展范围内没有障碍物！")
    
    fp_init = None
    while fp_init is None:
        fp_init = robot.get_flange_pose()
        time.sleep(0.1)
    fp_init_pose =list(fp_init.msg)   
    # 这是一个极其典型的“防无解”姿态：底座不动，肩膀往前抬一点，手肘弯曲90度，手腕自然朝前。
    safe_joints = [0.0, 0.5, 0.0, 1.0, 0.0, 0.0, 0.0]
    
    robot.move_j(safe_joints)
    end_effector.move_gripper_m(value=0.05,force=1.0)
    wait_for_motion(robot, timeout=25.0) 
    print("✅ 已到达安全数学姿态！")
    time.sleep(1.0)
    
    # ================= 3. 降速并获取基准坐标 =================
    # 将速度降回 2%，准备微动测试
    robot.set_speed_percent(2)
    print("🐢 已将运动速度降至 2%，确保微动测试绝对安全。")
    time.sleep(1.0)
    
    try:
        # 获取机械臂在安全姿态下的法兰位姿
        fp = None
        fw =None
        while fp is None:
            fp = robot.get_flange_pose()
            time.sleep(0.1)
        while fw is None:
            fw =robot.get_firmware()
        print(f"\n🤖 当前固件版本: {fw}")
        print(f"获取到初始位姿为：{fp.msg}")

        base_pose = list(fp.msg)
        print(f"\n📍 当前初始位姿锁定: X={base_pose[0]:.3f}, Y={base_pose[1]:.3f}, Z={base_pose[2]:.3f}")
        
        # # ================= 4. 开始交互式测试 =================
        #         # ---------------- 测试 基础点位运动 ----------------
        # input("\n👉 步骤 1：准备测试机械臂的基础点位运动\n   按回车键后，机械臂返回初始点位")

        # robot.move_p(
            
        # )
        # print("   正在移动...")
        # wait_for_motion(robot)

                # ---------------- 测试 +X 轴 ----------------
        input("\n👉 步骤 1：准备测试机械臂的【+X轴】。\n   按回车键后，机械臂将沿着自身的 +X 轴缓慢移动 20mm (0.02m)...")
        target_x = base_pose.copy()
        target_x[0] += 0.02
        robot.move_p(target_x)
        print("   正在移动...")
        wait_for_motion(robot)
        
        input("📝 观察它往哪个现实物理方向移动了？记录下来。\n   【按回车键让它退回安全原点】...")
        robot.move_p(base_pose)
        wait_for_motion(robot)

                # ---------------- 测试 +Z 轴 ----------------
        input("\n👉 步骤 3：准备测试机械臂的【+Z轴】。\n   按回车键后，机械臂将沿着自身的 +Z 轴缓慢移动 20mm (0.02m)...")
        target_z = base_pose.copy()
        target_x[2] += 0.02
        robot.move_p(target_x)
        print("   正在移动...")
        wait_for_motion(robot)
        
        input("📝 观察它往哪个现实物理方向移动了？记录下来。\n   【按回车键让它退回安全原点】...")
        robot.move_p(base_pose)
        wait_for_motion(robot)


                # ---------------- 测试 +Y 轴 ----------------
        input("\n👉 步骤 2：准备测试机械臂的【+Y轴】。\n   按回车键后，机械臂将沿着自身的 +Y 轴缓慢移动 20mm (0.02m)...")
        target_y = base_pose.copy()
        target_y[1] += 0.02
        robot.move_p(target_y)
        print("   正在移动...")
        wait_for_motion(robot)
        
        input("📝 观察它往哪个现实物理方向移动了？记录下来。\n   【按回车键让它退回安全原点】...")
        robot.move_p(base_pose)
        wait_for_motion(robot)
        

        




        print("\n🎉 测试全部完成！")
        print("请把你的肉眼观察结果（例如：发指令走+X，实际往左边走了）发给我，这下肯定能出正确矩阵了！")

    except KeyboardInterrupt:
        print("\n\n🛑 收到退出指令，准备下电...")
    finally:
        # ================= 5. 安全退出 =================
        print("正在断开连接...")
        robot.disable()
        robot.disconnect()
        print("✅ 已安全下电！")

if __name__ == "__main__":
    main()