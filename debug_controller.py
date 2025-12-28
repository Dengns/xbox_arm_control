#!/usr/bin/env python3
"""调试 Xbox 控制器输入"""

import pygame
import time

pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    print("未检测到控制器！")
    exit(1)

joystick = pygame.joystick.Joystick(0)
joystick.init()

print(f"检测到控制器: {joystick.get_name()}")
print(f"轴数量: {joystick.get_numaxes()}")
print(f"按钮数量: {joystick.get_numbuttons()}")
print("\n开始读取输入（按 Ctrl+C 退出）...")
print("请移动摇杆和按按钮测试\n")

try:
    while True:
        pygame.event.pump()

        # 读取所有轴
        print("\r轴值: ", end='')
        for i in range(joystick.get_numaxes()):
            value = joystick.get_axis(i)
            print(f"[{i}]={value:+.3f} ", end='')

        # 读取按钮
        print("  按钮: ", end='')
        for i in range(min(8, joystick.get_numbuttons())):
            if joystick.get_button(i):
                print(f"[{i}] ", end='')

        # 读取方向键
        if joystick.get_numhats() > 0:
            hat = joystick.get_hat(0)
            print(f"  方向键:{hat}", end='')

        print("    ", end='', flush=True)

        time.sleep(0.05)

except KeyboardInterrupt:
    print("\n\n退出。")
    pygame.quit()
