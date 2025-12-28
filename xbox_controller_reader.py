#!/usr/bin/env python3
"""
Xbox 控制器数据读取器
使用 pygame 库读取控制器输入并实时显示
"""

import pygame
import sys

def main():
    # 初始化 pygame
    pygame.init()
    pygame.joystick.init()

    # 检查控制器数量
    joystick_count = pygame.joystick.get_count()
    if joystick_count == 0:
        print("错误: 未检测到游戏控制器")
        return

    print(f"检测到 {joystick_count} 个控制器")

    # 初始化第一个控制器
    joystick = pygame.joystick.Joystick(0)
    joystick.init()

    # 打印控制器信息
    print("\n" + "="*60)
    print(f"控制器名称: {joystick.get_name()}")
    print(f"轴数量: {joystick.get_numaxes()}")
    print(f"按钮数量: {joystick.get_numbuttons()}")
    print(f"帽键数量: {joystick.get_numhats()}")
    print("="*60)
    print("\n按 Ctrl+C 退出\n")

    clock = pygame.time.Clock()

    try:
        while True:
            # 处理事件
            pygame.event.pump()

            # 清屏 (打印多行空白以模拟清屏效果)
            print("\033[2J\033[H", end="")

            print("="*60)
            print(f"Xbox 控制器实时数据 - {joystick.get_name()}")
            print("="*60)

            # 读取摇杆轴
            print("\n【摇杆/扳机】")
            axis_names = [
                "左摇杆 X轴",
                "左摇杆 Y轴",
                "左扳机 (LT)",
                "右摇杆 X轴",
                "右摇杆 Y轴",
                "右扳机 (RT)"
            ]

            for i in range(min(joystick.get_numaxes(), len(axis_names))):
                value = joystick.get_axis(i)
                # 创建可视化进度条
                bar_length = 30
                if i == 2 or i == 5:  # 扳机 (0到1)
                    normalized = (value + 1) / 2
                    filled = int(normalized * bar_length)
                    bar = "█" * filled + "░" * (bar_length - filled)
                    print(f"  {axis_names[i]:15s}: {value:6.3f} [{bar}]")
                else:  # 摇杆 (-1到1)
                    normalized = (value + 1) / 2
                    filled = int(normalized * bar_length)
                    bar = "█" * filled + "░" * (bar_length - filled)
                    print(f"  {axis_names[i]:15s}: {value:6.3f} [{bar}]")

            # 读取按钮
            print("\n【按钮状态】")
            button_names = [
                "A", "B", "X", "Y",
                "LB", "RB",
                "View", "Menu",
                "左摇杆按下", "右摇杆按下",
                "Xbox"
            ]

            buttons_pressed = []
            for i in range(joystick.get_numbuttons()):
                if joystick.get_button(i):
                    name = button_names[i] if i < len(button_names) else f"按钮{i}"
                    buttons_pressed.append(name)

            if buttons_pressed:
                print(f"  按下: {', '.join(buttons_pressed)}")
            else:
                print("  无按钮按下")

            # 读取方向键 (hat)
            print("\n【方向键 (D-Pad)】")
            if joystick.get_numhats() > 0:
                hat = joystick.get_hat(0)
                direction = ""
                if hat[1] == 1:
                    direction = "上"
                elif hat[1] == -1:
                    direction = "下"

                if hat[0] == 1:
                    direction += "右" if direction else "右"
                elif hat[0] == -1:
                    direction += "左" if direction else "左"

                if direction:
                    print(f"  方向: {direction} {hat}")
                else:
                    print(f"  方向: 居中 {hat}")

            print("="*60)
            print("按 Ctrl+C 退出")

            # 控制刷新率
            clock.tick(30)  # 30 FPS

    except KeyboardInterrupt:
        print("\n\n程序退出")
    finally:
        pygame.quit()

if __name__ == "__main__":
    main()
