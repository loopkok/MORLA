import time
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, NeroFW

cfg = create_agx_arm_config(
    robot=ArmModel.NERO, 
    firmeware_version=NeroFW.DEFAULT,  # 使用你上次代码里的 DEFAULT 固件版本
    channel="can_nero_left"           # 使用你上次代码里的通道名称
)
robot =AgxArmFactory.create_arm(cfg)
end_effector = robot.init_effector(robot.OPTIONS.EFFECTOR.AGX_GRIPPER)
robot.connect()

time.sleep(1.0)

print("effector_is_ok=",end_effector.is_ok())

while True:
    gs = end_effector.get_gripper_status()
    if gs is not None:
        print("value=",gs.msg.value,"model=",gs.msg.mode,"force=",gs.msg.force)
        break
    time.sleep(0.05)
while True:
    end_effector.move_gripper_deg(value=5.0,force=0.05)
    time.sleep(2.0)
