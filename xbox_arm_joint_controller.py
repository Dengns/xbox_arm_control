#!/usr/bin/env python3
"""
Xbox控制器控制Piper机械臂 - 关节控制模式
使用关节角度控制，LB键作为使能开关

==============================================================================
                            操作说明
==============================================================================

启动方式:
    python3 xbox_arm_joint_controller.py              # 默认使用 can0
    python3 xbox_arm_joint_controller.py --can can1   # 指定 CAN 端口
    python3 xbox_arm_joint_controller.py -c can_piper # 简写形式

控制方式:
    ┌─────────────────────────────────────────────────────────────────────┐
    │  【重要】按住 LB 键才能使能控制，松开 LB 立即停止                    │
    └─────────────────────────────────────────────────────────────────────┘

    摇杆控制:
        左摇杆 X 轴  → J1 底座旋转
        左摇杆 Y 轴  → J2 肩关节
        右摇杆 X 轴  → J3 肘关节
        右摇杆 Y 轴  → J4 腕关节1

    扳机控制:
        LT 扳机      → J5 腕关节2 正向
        RT 扳机      → J5 腕关节2 负向

    按钮控制:
        A 按钮       → J6 末端旋转 正向
        B 按钮       → J6 末端旋转 负向
        LB 按钮      → 【使能开关】按住才能控制

    十字键控制 (增量模式):
        十字键 ↑     → 按住持续张开夹爪
        十字键 ↓     → 按住持续闭合夹爪
        松开         → 停止

    退出:
        Ctrl + C     → 安全退出程序

==============================================================================
"""

import pygame
import time
import sys
import argparse
from typing import Optional, List
from piper_sdk import C_PiperInterface_V2


class ControlConfig:
    """控制参数配置"""

    # 关节速度缩放因子 (度/秒)
    JOINT_VELOCITY_SCALE = 20.0  # 摇杆满偏时的角速度

    # 死区设置
    JOYSTICK_DEADZONE = 0.1  # 10%
    TRIGGER_DEADZONE = 0.05  # 5%

    # 关节角度限制 (度)
    JOINT_LIMITS = {
        1: (-150, 150),
        2: (0, 180),
        3: (-170, 0),
        4: (-100, 100),
        5: (-70, 70),
        6: (-180, 180)
    }

    # 夹爪参数
    GRIPPER_MAX_POS = 80000     # 最大开度 80mm
    GRIPPER_MIN_POS = 0         # 最小开度 0mm
    GRIPPER_SPEED = 1000
    GRIPPER_STEP = 1500         # 每次循环增量

    # 控制频率
    CONTROL_FREQUENCY = 50  # Hz


class ControllerState:
    """控制器状态"""

    def __init__(self):
        self.enabled = False
        self.current_joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # 当前关节角度 (度)
        self.target_joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]   # 目标关节角度 (度)
        self.joint_velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # 关节速度
        self.last_update_time = time.time()
        self.gripper_state = "idle"
        self.gripper_pos = 0
        self.initialized = False


class XboxArmJointController:
    """Xbox控制器关节控制模式"""

    def __init__(self, can_port: str = "can0"):
        self.can_port = can_port
        self.config = ControlConfig()
        self.state = ControllerState()

        self.piper: Optional[C_PiperInterface_V2] = None
        self.joystick: Optional[pygame.joystick.Joystick] = None

        self.should_exit = False
        self.frame_count = 0

    def apply_deadzone(self, value: float, deadzone: float) -> float:
        """应用死区"""
        if abs(value) < deadzone:
            return 0.0
        sign = 1 if value > 0 else -1
        normalized = (abs(value) - deadzone) / (1.0 - deadzone)
        return sign * normalized

    def initialize_controller(self) -> bool:
        """初始化Xbox控制器"""
        try:
            pygame.init()
            pygame.joystick.init()

            if pygame.joystick.get_count() == 0:
                print("[ERROR] 未检测到游戏控制器")
                return False

            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()

            print(f"[SUCCESS] 检测到控制器: {self.joystick.get_name()}")
            return True

        except Exception as e:
            print(f"[ERROR] 控制器初始化失败: {e}")
            return False

    def initialize_arm(self) -> bool:
        """初始化机械臂"""
        try:
            self.piper = C_PiperInterface_V2(self.can_port)
            self.piper.ConnectPort()
            time.sleep(0.1)

            # 使能机械臂
            print("[INFO] 使能机械臂...")
            for _ in range(30):
                self.piper.EnableArm(7)
                time.sleep(0.1)

                low = self.piper.GetArmLowSpdInfoMsgs()
                if (low.motor_1.foc_status.driver_enable_status and
                    low.motor_2.foc_status.driver_enable_status and
                    low.motor_3.foc_status.driver_enable_status and
                    low.motor_4.foc_status.driver_enable_status and
                    low.motor_5.foc_status.driver_enable_status and
                    low.motor_6.foc_status.driver_enable_status):
                    break
            else:
                print("[WARNING] 部分电机可能未使能")

            # 初始化夹爪
            self.piper.GripperCtrl(0, self.config.GRIPPER_SPEED, 0x01, 0)
            time.sleep(0.1)

            # 读取当前关节角度
            self.update_current_joints()
            # 用当前角度初始化目标角度
            self.state.target_joints = self.state.current_joints.copy()
            self.state.initialized = True

            print(f"[SUCCESS] 机械臂初始化完成")
            print(f"          当前关节: J1={self.state.current_joints[0]:.1f}° "
                  f"J2={self.state.current_joints[1]:.1f}° "
                  f"J3={self.state.current_joints[2]:.1f}° "
                  f"J4={self.state.current_joints[3]:.1f}° "
                  f"J5={self.state.current_joints[4]:.1f}° "
                  f"J6={self.state.current_joints[5]:.1f}°")

            return True

        except Exception as e:
            print(f"[ERROR] 机械臂初始化失败: {e}")
            return False

    def initialize(self) -> bool:
        """初始化所有组件"""
        print("=" * 60)
        print("   Piper机械臂 Xbox控制器 - 关节控制模式")
        print("=" * 60)

        print("[1/3] 正在检测Xbox控制器...")
        if not self.initialize_controller():
            return False

        print(f"[2/3] 正在连接CAN总线 ({self.can_port})...")
        print("[3/3] 正在初始化机械臂...")
        if not self.initialize_arm():
            return False

        print("=" * 60)
        print("系统就绪！")
        print("  - 按住 LB 键使能控制")
        print("  - 左摇杆: J1/J2  右摇杆: J3/J4")
        print("  - LT/RT: J5  A/B: J6")
        print("  - 十字键: 夹爪")
        print("  - Ctrl+C 安全退出")
        print("=" * 60)

        return True

    def update_current_joints(self):
        """读取当前关节角度"""
        try:
            joint = self.piper.GetArmJointMsgs()
            # 转换为度 (原始单位是 0.001度)
            self.state.current_joints = [
                joint.joint_state.joint_1 * 0.001,
                joint.joint_state.joint_2 * 0.001,
                joint.joint_state.joint_3 * 0.001,
                joint.joint_state.joint_4 * 0.001,
                joint.joint_state.joint_5 * 0.001,
                joint.joint_state.joint_6 * 0.001
            ]
        except Exception as e:
            print(f"[ERROR] 读取关节角度失败: {e}")

    def read_controller_inputs(self) -> dict:
        """读取控制器输入"""
        pygame.event.pump()

        left_x = self.apply_deadzone(self.joystick.get_axis(0), self.config.JOYSTICK_DEADZONE)
        left_y = self.apply_deadzone(self.joystick.get_axis(1), self.config.JOYSTICK_DEADZONE)
        right_x = self.apply_deadzone(self.joystick.get_axis(3), self.config.JOYSTICK_DEADZONE)
        right_y = self.apply_deadzone(self.joystick.get_axis(4), self.config.JOYSTICK_DEADZONE)

        lt_raw = (self.joystick.get_axis(2) + 1) / 2
        rt_raw = (self.joystick.get_axis(5) + 1) / 2
        lt = self.apply_deadzone(lt_raw, self.config.TRIGGER_DEADZONE)
        rt = self.apply_deadzone(rt_raw, self.config.TRIGGER_DEADZONE)

        button_a = self.joystick.get_button(0)
        button_b = self.joystick.get_button(1)
        button_lb = self.joystick.get_button(4)

        hat = self.joystick.get_hat(0) if self.joystick.get_numhats() > 0 else (0, 0)

        return {
            'left_x': left_x,
            'left_y': left_y,
            'right_x': right_x,
            'right_y': right_y,
            'lt': lt,
            'rt': rt,
            'button_a': button_a,
            'button_b': button_b,
            'button_lb': button_lb,
            'dpad_up': hat[1] == 1,
            'dpad_down': hat[1] == -1
        }

    def calculate_target_joints(self, inputs: dict, dt: float):
        """计算目标关节角度"""
        scale = self.config.JOINT_VELOCITY_SCALE

        # 计算各关节速度
        vel = [
            inputs['left_x'] * scale,                          # J1
            -inputs['left_y'] * scale,                         # J2 (反转)
            inputs['right_x'] * scale,                         # J3
            -inputs['right_y'] * scale,                        # J4 (反转)
            (inputs['lt'] - inputs['rt']) * scale,             # J5
            (inputs['button_a'] - inputs['button_b']) * scale  # J6
        ]

        self.state.joint_velocity = vel

        # 累积目标角度
        for i in range(6):
            self.state.target_joints[i] += vel[i] * dt
            # 限制在关节限位内
            limits = self.config.JOINT_LIMITS[i + 1]
            self.state.target_joints[i] = max(limits[0], min(limits[1], self.state.target_joints[i]))

    def send_joint_command(self):
        """发送关节控制命令"""
        try:
            # 转换为 0.001度 单位
            j1 = int(self.state.target_joints[0] * 1000)
            j2 = int(self.state.target_joints[1] * 1000)
            j3 = int(self.state.target_joints[2] * 1000)
            j4 = int(self.state.target_joints[3] * 1000)
            j5 = int(self.state.target_joints[4] * 1000)
            j6 = int(self.state.target_joints[5] * 1000)

            # 设置关节控制模式
            self.piper.MotionCtrl_2(0x01, 0x01, 50, 0x00)
            # 发送关节命令
            self.piper.JointCtrl(j1, j2, j3, j4, j5, j6)

        except Exception as e:
            print(f"[ERROR] 发送关节命令失败: {e}")

    def handle_gripper(self, inputs: dict):
        """处理夹爪控制"""
        try:
            if inputs['dpad_up']:
                self.state.gripper_state = "opening"
                self.state.gripper_pos += self.config.GRIPPER_STEP
                if self.state.gripper_pos > self.config.GRIPPER_MAX_POS:
                    self.state.gripper_pos = self.config.GRIPPER_MAX_POS
                self.piper.GripperCtrl(int(self.state.gripper_pos), self.config.GRIPPER_SPEED, 0x01, 0)
            elif inputs['dpad_down']:
                self.state.gripper_state = "closing"
                self.state.gripper_pos -= self.config.GRIPPER_STEP
                if self.state.gripper_pos < self.config.GRIPPER_MIN_POS:
                    self.state.gripper_pos = self.config.GRIPPER_MIN_POS
                self.piper.GripperCtrl(int(self.state.gripper_pos), self.config.GRIPPER_SPEED, 0x01, 0)
            else:
                if self.state.gripper_state != "idle":
                    self.state.gripper_state = "idle"
        except Exception as e:
            print(f"[ERROR] 夹爪控制失败: {e}")

    def update_display(self):
        """更新状态显示"""
        j = self.state.target_joints
        v = self.state.joint_velocity
        enabled_str = "已使能" if self.state.enabled else "未使能"
        gripper_mm = self.state.gripper_pos / 1000.0

        status = (f"[{enabled_str}] "
                  f"J1={j[0]:6.1f} J2={j[1]:6.1f} J3={j[2]:6.1f} "
                  f"J4={j[3]:6.1f} J5={j[4]:6.1f} J6={j[5]:6.1f} | "
                  f"夹爪:{gripper_mm:.1f}mm")

        print(f"\r{status}", end='', flush=True)

    def run(self):
        """主控制循环"""
        clock = pygame.time.Clock()

        try:
            while not self.should_exit:
                try:
                    inputs = self.read_controller_inputs()
                    lb_pressed = inputs['button_lb']

                    # 夹爪控制不需要按 LB
                    self.handle_gripper(inputs)

                    if lb_pressed:
                        if not self.state.enabled:
                            print("\n[INFO] 控制器已使能")
                            self.state.enabled = True
                            # 重新读取当前角度作为起点
                            self.update_current_joints()
                            self.state.target_joints = self.state.current_joints.copy()

                        dt = time.time() - self.state.last_update_time
                        self.calculate_target_joints(inputs, dt)
                        self.send_joint_command()

                    else:
                        if self.state.enabled:
                            print("\n[INFO] 控制器已失能")
                            self.state.enabled = False

                    self.state.last_update_time = time.time()
                    self.frame_count += 1

                    if self.frame_count % 5 == 0:
                        self.update_display()

                except Exception as e:
                    print(f"\n[ERROR] 控制循环异常: {e}")

                clock.tick(self.config.CONTROL_FREQUENCY)

        except KeyboardInterrupt:
            print("\n\n[INFO] 用户请求退出...")
        finally:
            self.shutdown()

    def shutdown(self):
        """安全关闭"""
        print("\n" + "=" * 60)
        print("正在安全关闭...")

        try:
            if self.piper:
                self.piper.GripperCtrl(0, 1000, 0x01, 0)
                time.sleep(0.1)
                self.piper.DisableArm(7)
        except:
            pass

        if self.joystick:
            pygame.quit()

        print("系统已安全关闭")
        print("=" * 60)


def main():
    parser = argparse.ArgumentParser(
        description="Xbox控制器控制Piper机械臂 - 关节控制模式",
        formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("-c", "--can", type=str, default="can0", help="CAN 端口名称 (默认: can0)")
    args = parser.parse_args()

    print(f"[INFO] 使用 CAN 端口: {args.can}")
    controller = XboxArmJointController(args.can)

    if controller.initialize():
        controller.run()
    else:
        print("[FAILED] 系统初始化失败")
        sys.exit(1)


if __name__ == "__main__":
    main()
