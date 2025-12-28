#!/usr/bin/env python3
"""测试 Piper SDK 连接"""

import time
from piper_sdk import C_PiperInterface_V2

print("正在尝试连接 CAN 总线 (can_piper)...")

try:
    # 方法1: 使用默认参数
    print("\n方法1: 使用 C_PiperInterface_V2(can_name='can_piper')")
    piper = C_PiperInterface_V2(can_name='can_piper')
    piper.ConnectPort()
    time.sleep(0.5)

    # 尝试读取状态
    print("尝试读取机械臂状态...")
    end_pose = piper.GetArmEndPoseMsgs()
    print(f"成功! 当前位姿: X={end_pose.end_pose.X_axis}, Y={end_pose.end_pose.Y_axis}, Z={end_pose.end_pose.Z_axis}")

    print("\n连接成功!")

except Exception as e:
    print(f"连接失败: {e}")
    print(f"错误类型: {type(e)}")
    import traceback
    traceback.print_exc()
