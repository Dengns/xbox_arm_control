#!/usr/bin/env python3
"""
Xbox 控制器原始数据读取器
直接读取 /dev/input/js0 设备文件 (无需pygame)
"""

import struct
import os
import sys

# 事件格式: time (4字节), value (2字节), type (1字节), number (1字节)
EVENT_FORMAT = 'IhBB'
EVENT_SIZE = struct.calcsize(EVENT_FORMAT)

# 事件类型
JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS = 0x02
JS_EVENT_INIT = 0x80

def main():
    device = '/dev/input/js0'

    if not os.path.exists(device):
        print(f"错误: 设备 {device} 不存在")
        print("请确保控制器已连接")
        return

    print("="*60)
    print(f"正在读取: {device}")
    print("="*60)

    # 状态存储
    axes = {}
    buttons = {}

    axis_names = {
        0: "左摇杆 X",
        1: "左摇杆 Y",
        2: "左扳机 (LT)",
        3: "右摇杆 X",
        4: "右摇杆 Y",
        5: "右扳机 (RT)"
    }

    button_names = {
        0: "A", 1: "B", 2: "X", 3: "Y",
        4: "LB", 5: "RB",
        6: "View", 7: "Menu",
        8: "左摇杆按下", 9: "右摇杆按下",
        10: "Xbox"
    }

    try:
        with open(device, 'rb') as js_device:
            print("控制器已连接，开始读取数据...")
            print("按 Ctrl+C 退出\n")

            while True:
                # 读取事件
                event = js_device.read(EVENT_SIZE)

                if event:
                    time, value, evt_type, number = struct.unpack(EVENT_FORMAT, event)

                    # 忽略初始化事件
                    if evt_type & JS_EVENT_INIT:
                        evt_type &= ~JS_EVENT_INIT
                        if evt_type == JS_EVENT_AXIS:
                            axes[number] = value
                        elif evt_type == JS_EVENT_BUTTON:
                            buttons[number] = value
                        continue

                    # 处理轴事件
                    if evt_type == JS_EVENT_AXIS:
                        axes[number] = value
                        normalized = value / 32767.0
                        name = axis_names.get(number, f"轴{number}")

                        # 创建简单的可视化
                        bar_length = 20
                        filled = int((normalized + 1) / 2 * bar_length)
                        bar = "█" * filled + "░" * (bar_length - filled)

                        print(f"[轴 {number:2d}] {name:15s}: {normalized:7.3f} [{bar}] (原始: {value:6d})")

                    # 处理按钮事件
                    elif evt_type == JS_EVENT_BUTTON:
                        buttons[number] = value
                        name = button_names.get(number, f"按钮{number}")
                        status = "按下" if value else "释放"
                        print(f"[按钮{number:2d}] {name:15s}: {status}")

    except KeyboardInterrupt:
        print("\n\n程序退出")
    except PermissionError:
        print(f"\n错误: 没有权限访问 {device}")
        print("请尝试运行: sudo python3 xbox_controller_raw.py")
    except Exception as e:
        print(f"\n错误: {e}")

if __name__ == "__main__":
    main()
