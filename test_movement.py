#!/usr/bin/env python3
"""测试机械臂能否响应运动命令"""

import time
from piper_sdk import C_PiperInterface_V2

print("正在连接机械臂...")
piper = C_PiperInterface_V2(can_name='can_piper')
piper.ConnectPort()
time.sleep(0.1)

# 读取当前位姿
print("\n读取当前位姿...")
end_pose = piper.GetArmEndPoseMsgs()
current_x = end_pose.end_pose.X_axis
current_y = end_pose.end_pose.Y_axis
current_z = end_pose.end_pose.Z_axis
current_rx = end_pose.end_pose.RX_axis
current_ry = end_pose.end_pose.RY_axis
current_rz = end_pose.end_pose.RZ_axis

print(f"当前位姿: X={current_x} Y={current_y} Z={current_z}")
print(f"          RX={current_rx} RY={current_ry} RZ={current_rz}")

# 使能机械臂
print("\n使能机械臂...")
for _ in range(10):
    piper.EnableArm(7)
    time.sleep(0.1)
    low_spd_info = piper.GetArmLowSpdInfoMsgs()
    if (low_spd_info.motor_1.foc_status.driver_enable_status and
        low_spd_info.motor_2.foc_status.driver_enable_status):
        print("机械臂已使能")
        break
else:
    print("使能失败！")
    exit(1)

# 测试1: 使用 MotionCtrl_1 (关节空间运动)
print("\n测试方法1: 使用 MotionCtrl_1 关节运动...")
print("设置运动模式为关节运动...")
piper.MotionCtrl_1(0x00, 0x01, 30, 0)  # 模式0x00=关节空间, 0x01=使能, 30%速度
time.sleep(0.2)

# 读取当前关节角度
joint_msgs = piper.GetArmJointMsgs()
print(f"当前关节角度: [{joint_msgs.joint_state.joint_1/1000:.1f}, "
      f"{joint_msgs.joint_state.joint_2/1000:.1f}, "
      f"{joint_msgs.joint_state.joint_3/1000:.1f}, "
      f"{joint_msgs.joint_state.joint_4/1000:.1f}, "
      f"{joint_msgs.joint_state.joint_5/1000:.1f}, "
      f"{joint_msgs.joint_state.joint_6/1000:.1f}]")

# 尝试移动关节1 +5度
print("\n发送命令: 关节1 移动 +5度...")
target_j1 = joint_msgs.joint_state.joint_1 + 5000  # +5度
piper.JointCtrl(
    target_j1,
    joint_msgs.joint_state.joint_2,
    joint_msgs.joint_state.joint_3,
    joint_msgs.joint_state.joint_4,
    joint_msgs.joint_state.joint_5,
    joint_msgs.joint_state.joint_6
)

# 等待并监控
print("等待3秒，监控位置变化...")
for i in range(6):
    time.sleep(0.5)
    joint_msgs = piper.GetArmJointMsgs()
    print(f"  [{i*0.5:.1f}s] 关节1角度: {joint_msgs.joint_state.joint_1/1000:.2f}°")

# 测试2: 使用 MotionCtrl_2 (笛卡尔空间运动)
print("\n测试方法2: 使用 MotionCtrl_2 笛卡尔运动...")
print("设置运动模式为笛卡尔空间...")
piper.MotionCtrl_2(0x01, 0x00, 30, 0x00)  # 0x01=笛卡尔空间
time.sleep(0.2)

# 读取当前末端位姿
end_pose = piper.GetArmEndPoseMsgs()
current_x = end_pose.end_pose.X_axis
current_y = end_pose.end_pose.Y_axis
current_z = end_pose.end_pose.Z_axis
print(f"当前末端位姿: X={current_x/1000:.1f} Y={current_y/1000:.1f} Z={current_z/1000:.1f}")

# 尝试移动 Z轴 +10mm
print("\n发送命令: Z轴 +10mm...")
piper.EndPoseCtrl(
    current_x,
    current_y,
    current_z + 10000,  # +10mm
    end_pose.end_pose.RX_axis,
    end_pose.end_pose.RY_axis,
    end_pose.end_pose.RZ_axis
)

# 等待并监控
print("等待3秒，监控位置变化...")
for i in range(6):
    time.sleep(0.5)
    end_pose = piper.GetArmEndPoseMsgs()
    print(f"  [{i*0.5:.1f}s] Z轴: {end_pose.end_pose.Z_axis/1000:.2f}mm")

print("\n失能机械臂...")
piper.DisableArm(7)

print("测试完成！")
