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
        channel="can_nero_left"           # 使用你上次代码里的通道名称
    )

    robot = AgxArmFactory.create_arm(cfg)
    robot.connect()

    print("⏳ 正在使能机械臂...")
    while not robot.enable():
        time.sleep(0.1)
        
    print("✅ 使能成功！")

    robot.set_speed_percent(2)
    print("🐢 已将运动速度降至 2%，确保微动测试绝对安全。")
    time.sleep(1.0)

    try:
        safe_joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    
        robot.move_j(safe_joints)
        wait_for_motion(robot, timeout=25.0) 
        print("✅ 已到达安全数学姿态！")
        time.sleep(1.0)
        robot.move_p([-0.45,-0.0,0.45,-1.5708,0.0,-3.14159])
        wait_for_motion(robot=robot,timeout=25)

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