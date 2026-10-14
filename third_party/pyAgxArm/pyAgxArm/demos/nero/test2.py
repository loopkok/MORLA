import time
import numpy as np
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, NeroFW

def get_stable_pose(robot):
    """获取当前法兰位姿（带重试机制以确保拿到有效数据）"""
    while True:
        mja = robot.get_leader_joint_angles()
        if mja is not None:
            return np.array(robot.fk(mja.msg))
        time.sleep(0.05)

def analyze_dominant_axis(delta_pos):
    """分析增量中变化最大的轴，并返回轴名称和正负号"""
    axis_names = ['X', 'Y', 'Z']
    max_idx = np.argmax(np.abs(delta_pos)) # 找到绝对值最大的变化量
    max_val = delta_pos[max_idx]
    sign = "+" if max_val > 0 else "-"
    return f"{sign}{axis_names[max_idx]}", max_val

def main():
    # ================= 1. 配置与连接 =================
    cfg = create_agx_arm_config(
        robot=ArmModel.NERO, 
        firmeware_version=NeroFW.DEFAULT,  # ⚠️ 确认固件版本
        channel="can_nero_left" \
        ""                  # ⚠️ 确认CAN通道
    )
    
    robot = AgxArmFactory.create_arm(cfg)
    robot.connect()
    
    # ================= 2. 使能与零力模式 =================
    print("⏳ 正在使能机械臂...")
    while not robot.enable():
        time.sleep(0.1)
        
    print("✅ 使能成功！正在开启【零力拖动模式】...")
    robot.set_leader_mode()
    time.sleep(1.0)
    
    try:
        # ================= 3. 状态机向导引导 =================
        print("\n" + "="*50)
        print(" 🚀 Nero 侧装双臂坐标系自动标定向导 (XYZ完整版) 🚀")
        print("="*50)
        
        # --- 测试 X 轴 (前挥) ---
        input("\n👉 步骤 1：让机械臂自然垂直下落，保持静止。\n   【准备好后，按回车键记录零点】...")
        p_zero_x = get_stable_pose(robot)
        
        input("\n👉 步骤 2：像人手一样【往前挥】（约20厘米）并托住保持静止。\n   【保持住，按回车键记录】...")
        p_fwd = get_stable_pose(robot)
        delta_fwd = p_fwd[:3] - p_zero_x[:3]
        fwd_axis, _ = analyze_dominant_axis(delta_fwd)
        print(f"📍 [记录成功] 前挥动作对应机械臂 -> {fwd_axis} 轴")

        # --- 测试 Y 轴 (右挥) ---
        input("\n👉 步骤 3：让机械臂回到自然垂直下落状态。\n   【准备好后，按回车键重新记录零点】...")
        p_zero_y = get_stable_pose(robot)
        
        input("\n👉 步骤 4：像人手一样【往外/往右挥】（约20厘米）并托住保持静止。\n   【保持住，按回车键记录】...")
        p_right = get_stable_pose(robot)
        delta_right = p_right[:3] - p_zero_y[:3]
        right_axis, _ = analyze_dominant_axis(delta_right)
        print(f"📍 [记录成功] 右挥动作对应机械臂 -> {right_axis} 轴")

        # --- 测试 Z 轴 (上抬) ---
        input("\n👉 步骤 5：让机械臂回到自然垂直下落状态。\n   【准备好后，按回车键最后一次记录零点】...")
        p_zero_z = get_stable_pose(robot)
        
        input("\n👉 步骤 6：像人手一样【往上抬起】（约20厘米，想象你在提裤子）并托住保持静止。\n   【保持住，按回车键记录】...")
        p_up = get_stable_pose(robot)
        delta_up = p_up[:3] - p_zero_z[:3]
        up_axis, _ = analyze_dominant_axis(delta_up)
        print(f"📍 [记录成功] 上抬动作对应机械臂 -> {up_axis} 轴")
        
        # ================= 4. 自动生成数据表格与结论 =================
        print("\n\n" + "*"*50)
        print(" 📊 标定数据报告与自动诊断 📊")
        print("*"*50)
        print(f"| 全局物理动作 | dX (m) | dY (m) | dZ (m) | 对应机械臂主轴 |")
        print(f"|:---|:---|:---|:---|:---|")
        print(f"| 往前挥 (+X)  | {delta_fwd[0]:+6.3f} | {delta_fwd[1]:+6.3f} | {delta_fwd[2]:+6.3f} | {fwd_axis} |")
        print(f"| 往右挥 (-Y)  | {delta_right[0]:+6.3f} | {delta_right[1]:+6.3f} | {delta_right[2]:+6.3f} | {right_axis} |")
        print(f"| 往上抬 (+Z)  | {delta_up[0]:+6.3f} | {delta_up[1]:+6.3f} | {delta_up[2]:+6.3f} | {up_axis} |")
        
        print("\n💡 【系统自动诊断结论】:")
        print(f"1. 人类的 正前方 (+X) 对应 Nero 机械臂的: {fwd_axis} 轴")
        print(f"2. 人类的 正右方 (-Y) 对应 Nero 机械臂的: {right_axis} 轴")
        print(f"3. 人类的 正上方 (+Z) 对应 Nero 机械臂的: {up_axis} 轴")
        print("\n👉 完美！现在请把这个表格发给我，你的 9 宫格映射矩阵直接秒出！")
        
    except KeyboardInterrupt:
        print("\n\n🛑 收到退出指令，准备下电...")
    finally:
        # ================= 5. 安全退出 =================
        robot.set_normal_mode()
        time.sleep(0.5)
        robot.disable()
        robot.disconnect()
        print("✅ 已安全断开连接并下电！")

if __name__ == "__main__":
    main()
# import time
# from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, NeroFW

# def main():
#     # ================= 1. 配置与连接 =================
#     cfg = create_agx_arm_config(
#         robot=ArmModel.NERO, 
#         firmeware_version=NeroFW.DEFAULT,  # ⚠️ 确认你的固件版本，若<=1.10改用 NeroFW.DEFAULT
#         channel="can_nero_right"                  # 确认你要测左臂还是右臂的通道
#     )
    
#     robot = AgxArmFactory.create_arm(cfg)
#     robot.connect()
    
#     # ================= 2. 使能与零力模式 =================
#     print("⏳ 正在使能机械臂，请手扶一下机械臂防止掉落...")
#     while not robot.enable():
#         time.sleep(0.1)
        
#     print("✅ 使能成功！正在开启【零力拖动模式】(Leader Mode)...")
#     robot.set_leader_mode()
#     time.sleep(1.0) # 等待模式切换生效
    
#     print("\n👉 测试步骤：")
#     print("  1. 让机械臂自然垂直下落，保持静止。")
#     print("  2. 像人手一样【往前挥】，观察 dX, dY, dZ 哪个在剧烈变动。")
#     print("  3. 像人手一样【往右挥】，观察哪个轴在剧烈变动。")
#     print("-" * 70)
    
#     init_pose = None
    
#     try:
#         # ================= 3. 实时读取与对比 =================
#         while True:
#             # 在 Leader 模式下，标准做法是读取 leader 关节角，然后算 FK
#             mja = robot.get_leader_joint_angles()
            
#             if mja is not None:
#                 # 使用 SDK 内置的正运动学算出现在手端的 XYZ
#                 current_pose = robot.fk(mja.msg) 
                
#                 if init_pose is None:
#                     init_pose = current_pose
#                     print(f"📍 记录初始垂下位姿: X={init_pose[0]:.3f}, Y={init_pose[1]:.3f}, Z={init_pose[2]:.3f} m\n")
#                     continue
                
#                 dx = current_pose[0] - init_pose[0]
#                 dy = current_pose[1] - init_pose[1]
#                 dz = current_pose[2] - init_pose[2]
                
#                 # 过滤微小抖动 (5毫米以内当做0)
#                 show_dx = dx if abs(dx) > 0.005 else 0.0
#                 show_dy = dy if abs(dy) > 0.005 else 0.0
#                 show_dz = dz if abs(dz) > 0.005 else 0.0
                
#                 print(f"实时增量 -> dX: {show_dx:+.3f} m | dY: {show_dy:+.3f} m | dZ: {show_dz:+.3f} m")
            
#             time.sleep(0.2)
            
#     except KeyboardInterrupt:
#         print("\n\n🛑 收到退出指令，准备下电...")
#     finally:
#         # ================= 4. 安全退出 =================
#         # 退出 Leader 模式必须先切回 Normal 模式，否则 disable() 可能会被忽略
#         robot.set_normal_mode()
#         time.sleep(0.5)
#         robot.disable()
#         robot.disconnect()
#         print("✅ 已安全断开连接并下电！")

# if __name__ == "__main__":
#     main()