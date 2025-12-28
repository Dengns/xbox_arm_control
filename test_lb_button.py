#!/usr/bin/env python3
"""测试 LB 按钮是否能被检测到"""

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
print(f"按钮数量: {joystick.get_numbuttons()}")
print("\n开始测试 LB 按钮（按钮索引4）...")
print("请按住 LB 键，看看是否能检测到\n")

try:
    while True:
        pygame.event.pump()

        # 读取 LB 按钮 (索引4)
        lb_pressed = joystick.get_button(4)

        # 读取所有按钮状态
        pressed_buttons = []
        for i in range(joystick.get_numbuttons()):
            if joystick.get_button(i):
                pressed_buttons.append(i)

        status = f"\rLB按钮(4): {'按下 ✓' if lb_pressed else '未按'} | "
        if pressed_buttons:
            status += f"当前按下的按钮: {pressed_buttons}"
        else:
            status += "当前按下的按钮: 无"

        print(status + "    ", end='', flush=True)

        time.sleep(0.05)

except KeyboardInterrupt:
    print("\n\n退出。")
    pygame.quit()
