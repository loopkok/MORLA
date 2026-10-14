import time
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, NeroFW

def test_and_diagnose_agx_gripper():
    print("=== 1. 初始化和连接机械臂与夹爪 ===")
    # 根据你的硬件配置：Nero 机械臂, CAN0 接口
    cfg = create_agx_arm_config(robot=ArmModel.NERO, firmeware_version=NeroFW.DEFAULT, channel="can_nero_right")
    robot = AgxArmFactory.create_arm(cfg)
    
    # 初始化末端执行器为 AgxGripper
    end_effector = robot.init_effector(robot.OPTIONS.EFFECTOR.AGX_GRIPPER)
    
    # 建立连接并开启底层数据读取线程
    robot.connect()
    print("已下发连接指令，等待底层通信建立 (1秒)...")
    time.sleep(1.0) 
    print("⏳ 正在使能机械臂...")
    while not robot.enable():
        time.sleep(0.1)
        
    print("✅ 使能成功！")

    print("\n=== 2. 检查通信状态 ===")
    is_ok = end_effector.is_ok()
    fps = end_effector.get_fps()
    print(f"通信状态 (is_ok): {is_ok}")
    print(f"接收频率 (fps): {fps:.1f} Hz")
    
    if not is_ok or fps == 0:
        print("❌ 致命错误：未检测到夹爪通信，请检查接线！脚本将继续尝试，但极大概率会失败。")

    print("\n=== 3. 深度读取夹爪底层驱动状态 (FOC Status) ===")
    # 循环尝试读取，留出数据缓冲时间
    status_found = False
    for _ in range(20):
        gs = end_effector.get_gripper_status()
        if gs is not None:
            status_found = True
            print(f"▶ 基础数据 -> 模式: {gs.msg.mode}, 行程/角度: {gs.msg.value:.4f}, 受力: {gs.msg.force:.2f} N")
            
            # --- 关键诊断信息：打印底层 FOC 状态位 ---
            foc = gs.msg.foc_status
            print("▶ 驱动器状态标志位 (请重点关注以下 False 的项):")
            print(f"  - 驱动使能 (Enable)   : {foc.driver_enable_status}  <-- 若为 False，电机没上电或被锁死")
            print(f"  - 回零完成 (Homing)   : {foc.homing_status}  <-- 若为 False，必须先执行标定才能动")
            print(f"  - 驱动错误 (Error)    : {foc.driver_error_status}  <-- 若为 True，说明硬件有报错")
            print(f"  - 传感器状态 (Sensor) : {foc.sensor_status}")
            print(f"  - 硬件报警 -> 电压低={foc.voltage_too_low}, 电机过温={foc.motor_overheating}, 驱动过流={foc.driver_overcurrent}")
            break
        time.sleep(0.1)
        
    if not status_found:
        print("⚠️ 警告：未能读取到任何夹爪实时状态反馈，但总线有心跳。可能是夹爪主板未启动。")

    # print("\n=== 4. 执行夹爪标定 (回零) ===")
    # print("发送强制标定指令，尝试激活夹爪运动权限...")
    if  end_effector.calibrate_gripper(timeout=5.0):
        print("✅ 夹爪标定指令成功执行！等待 2 秒让其寻找机械零点...")
        end_effector.move_gripper_m(value=0.0)
        time.sleep(2.0)
    else:
        print("❌ 夹爪拒绝标定，或标定超时。(如果下方运动测试失败，基本是这一步被拦截了)")

    print("\n=== 5. 夹爪运动控制测试 ===")
    # 标定完成后，尝试用稍微大一点的力矩去运动
    print("▶ 动作 1: 尝试张开夹爪到 5cm (0.05m)，力控设定为 1.5 N")
    end_effector.move_gripper_m(value=0.08, force=10)
    time.sleep(2.5) # 给足机械响应时间

    print("▶ 动作 2: 尝试完全闭合夹爪 (0.0m)，力控设定为 1.5 N")
    end_effector.move_gripper_m(value=0.0, force=10)
    time.sleep(2.5)

    print("\n=== 6. 驱动器失能测试 ===")
    print("正在尝试禁用夹爪电机...")
    if end_effector.disable_gripper():
        print("✅ 夹爪已成功失能 (测试完毕，当前可用手轻松掰动夹爪)。")
    else:
        print("❌ 夹爪失能指令失败或状态反馈未知。")

    print("\n=== 🎉 完整测试与诊断流程结束 ===")

if __name__ == "__main__":
    test_and_diagnose_agx_gripper()