#!/usr/bin/env python3
"""
Xbox控制器控制Piper机械臂
使用速度控制模式，LB键作为使能开关
"""

import pygame
import time
import sys
from typing import Optional, Tuple, List
from piper_sdk import C_PiperInterface_V2


class ControlConfig:
    """控制参数配置"""

    # 速度缩放因子
    LINEAR_VELOCITY_SCALE = 50.0   # mm/s (摇杆满偏时的速度)
    ANGULAR_VELOCITY_SCALE = 30.0  # deg/s

    # 速度限制
    MAX_LINEAR_VELOCITY = 100.0
    MAX_ANGULAR_VELOCITY = 60.0

    # 死区设置
    JOYSTICK_DEADZONE = 0.1  # 10%
    TRIGGER_DEADZONE = 0.05  # 5%

    # 位置限位
    POSITION_LIMITS = {
        'X': (0, 400),         # mm
        'Y': (-300, 300),
        'Z': (0, 500),
        'RX': (-120, 120),     # degrees
        'RY': (-70, 70),
        'RZ': (-150, 150)
    }

    # 软限位缓冲区
    SOFT_LIMIT_BUFFER = 20  # mm或degrees

    # 夹爪参数
    GRIPPER_OPEN_POS = 50000    # 50mm
    GRIPPER_CLOSE_POS = 0
    GRIPPER_SPEED = 1000
    GRIPPER_FORCE = 500

    # 控制频率
    CONTROL_FREQUENCY = 50  # Hz


class ControllerState:
    """控制器状态"""

    def __init__(self):
        self.enabled = False
        self.current_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # [X, Y, Z, RX, RY, RZ]
        self.target_velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.last_update_time = time.time()
        self.gripper_state = "idle"  # idle, opening, closing
        self.gripper_target = 0


class SafetyMonitor:
    """安全监控"""

    def __init__(self, config: ControlConfig):
        self.config = config
        self.last_can_communication = time.time()
        self.emergency_stop = False
        self.consecutive_errors = 0
        self.max_consecutive_errors = 5
        self.watchdog_timeout = 1.0  # 秒

    def check_safety(self, current_pose: List[float], target_pose: List[float]) -> Tuple[bool, str]:
        """
        综合安全检查

        Returns:
            (is_safe, error_message)
        """
        # 检查紧急停止
        if self.emergency_stop:
            return False, "紧急停止已触发"

        # 检查通信超时
        if time.time() - self.last_can_communication > self.watchdog_timeout:
            return False, "CAN通信超时"

        # 检查位置限位
        axis_names = ['X', 'Y', 'Z', 'RX', 'RY', 'RZ']
        for i, axis in enumerate(axis_names):
            limits = self.config.POSITION_LIMITS[axis]
            if not (limits[0] <= target_pose[i] <= limits[1]):
                return False, f"{axis}轴超出限位: {target_pose[i]:.1f}"

        # 检查软限位（接近边界时警告）
        for i, axis in enumerate(axis_names):
            limits = self.config.POSITION_LIMITS[axis]
            if target_pose[i] < limits[0] + self.config.SOFT_LIMIT_BUFFER:
                print(f"[WARNING] {axis}轴接近下限")
            elif target_pose[i] > limits[1] - self.config.SOFT_LIMIT_BUFFER:
                print(f"[WARNING] {axis}轴接近上限")

        return True, ""

    def trigger_emergency_stop(self):
        """触发紧急停止"""
        self.emergency_stop = True
        print("\n[EMERGENCY] 紧急停止触发！")


class XboxArmController:
    """Xbox控制器到机械臂的主控制器"""

    def __init__(self, can_port: str = "can_piper"):
        self.can_port = can_port
        self.config = ControlConfig()
        self.state = ControllerState()
        self.safety = SafetyMonitor(self.config)

        self.piper: Optional[C_PiperInterface_V2] = None
        self.joystick: Optional[pygame.joystick.Joystick] = None

        self.should_exit = False
        self.frame_count = 0

    def apply_deadzone(self, value: float, deadzone: float) -> float:
        """应用死区，避免摇杆漂移"""
        if abs(value) < deadzone:
            return 0.0

        # 重新映射，使死区外的值仍然覆盖完整范围
        sign = 1 if value > 0 else -1
        normalized = (abs(value) - deadzone) / (1.0 - deadzone)
        return sign * normalized

    def initialize_controller(self) -> bool:
        """初始化Xbox控制器"""
        try:
            pygame.init()
            pygame.joystick.init()

            joystick_count = pygame.joystick.get_count()
            if joystick_count == 0:
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
            # 连接CAN端口
            self.piper = C_PiperInterface_V2(self.can_port)
            self.piper.ConnectPort()
            time.sleep(0.1)

            # 设置控制模式
            self.piper.MotionCtrl_2(0x01, 0x00, 100, 0x00)
            time.sleep(0.1)

            # 使能机械臂
            if not self._enable_with_timeout(timeout=5.0):
                raise RuntimeError("机械臂使能超时")

            # 初始化夹爪
            self.piper.GripperCtrl(0, self.config.GRIPPER_SPEED, 0x01, 0)
            time.sleep(0.1)

            # 读取初始位姿
            end_pose = self.piper.GetArmEndPoseMsgs()
            self.state.current_pose = [
                end_pose.end_pose.X_axis / 1000.0,
                end_pose.end_pose.Y_axis / 1000.0,
                end_pose.end_pose.Z_axis / 1000.0,
                end_pose.end_pose.RX_axis / 1000.0,
                end_pose.end_pose.RY_axis / 1000.0,
                end_pose.end_pose.RZ_axis / 1000.0
            ]

            print(f"[SUCCESS] 机械臂初始化完成")
            print(f"          当前位姿: X={self.state.current_pose[0]:.1f} "
                  f"Y={self.state.current_pose[1]:.1f} "
                  f"Z={self.state.current_pose[2]:.1f} "
                  f"RX={self.state.current_pose[3]:.1f} "
                  f"RY={self.state.current_pose[4]:.1f} "
                  f"RZ={self.state.current_pose[5]:.1f}")
            return True

        except Exception as e:
            print(f"[ERROR] 机械臂初始化失败: {e}")
            return False

    def _enable_with_timeout(self, timeout: float = 5.0) -> bool:
        """使能机械臂并检测状态"""
        start_time = time.time()

        while True:
            elapsed_time = time.time() - start_time

            # 检查所有电机使能状态
            low_spd_info = self.piper.GetArmLowSpdInfoMsgs()
            enable_flag = (
                low_spd_info.motor_1.foc_status.driver_enable_status and
                low_spd_info.motor_2.foc_status.driver_enable_status and
                low_spd_info.motor_3.foc_status.driver_enable_status and
                low_spd_info.motor_4.foc_status.driver_enable_status and
                low_spd_info.motor_5.foc_status.driver_enable_status and
                low_spd_info.motor_6.foc_status.driver_enable_status
            )

            if enable_flag:
                return True

            # 发送使能命令
            self.piper.EnableArm(7)

            # 检查超时
            if elapsed_time > timeout:
                return False

            time.sleep(0.5)

    def initialize(self) -> bool:
        """初始化所有组件"""
        print("="*60)
        print("   Piper机械臂 Xbox控制器 v1.0")
        print("="*60)

        # 初始化控制器
        print("[1/4] 正在检测Xbox控制器...")
        if not self.initialize_controller():
            return False

        # 连接CAN总线
        print(f"[2/4] 正在连接CAN总线 ({self.can_port})...")

        # 初始化机械臂
        print("[3/4] 正在使能机械臂...")
        if not self.initialize_arm():
            return False

        print("[4/4] 系统初始化完成")
        print("="*60)
        print("系统就绪！")
        print("使用说明：")
        print("  - 按住 LB 键使能控制")
        print("  - 左摇杆控制 XY 平面")
        print("  - 右摇杆控制 Z 轴和 RZ 旋转")
        print("  - LT/RT 扳机控制 RY 俯仰")
        print("  - A/B 按钮控制 RX 翻滚")
        print("  - 十字键上/下控制夹爪")
        print("  - 按 Ctrl+C 安全退出")
        print("="*60)

        return True

    def read_controller_inputs(self) -> dict:
        """读取控制器输入并应用死区"""
        pygame.event.pump()

        # 读取摇杆
        left_x = self.apply_deadzone(
            self.joystick.get_axis(0),
            self.config.JOYSTICK_DEADZONE
        )
        left_y = self.apply_deadzone(
            self.joystick.get_axis(1),
            self.config.JOYSTICK_DEADZONE
        )
        right_x = self.apply_deadzone(
            self.joystick.get_axis(3),
            self.config.JOYSTICK_DEADZONE
        )
        right_y = self.apply_deadzone(
            self.joystick.get_axis(4),
            self.config.JOYSTICK_DEADZONE
        )

        # 读取扳机（-1到1，转换为0到1）
        lt_raw = (self.joystick.get_axis(2) + 1) / 2
        rt_raw = (self.joystick.get_axis(5) + 1) / 2

        lt = self.apply_deadzone(lt_raw, self.config.TRIGGER_DEADZONE)
        rt = self.apply_deadzone(rt_raw, self.config.TRIGGER_DEADZONE)

        # 读取按钮
        button_a = self.joystick.get_button(0)
        button_b = self.joystick.get_button(1)
        button_lb = self.joystick.get_button(4)

        # 读取方向键
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

    def calculate_target_pose(self, inputs: dict, dt: float) -> List[float]:
        """根据控制器输入计算目标位姿"""
        current = self.state.current_pose.copy()

        # 计算速度 (mm/s 或 deg/s)
        vel_x = inputs['left_x'] * self.config.LINEAR_VELOCITY_SCALE
        vel_y = -inputs['left_y'] * self.config.LINEAR_VELOCITY_SCALE  # Y轴反转
        vel_z = -inputs['right_y'] * self.config.LINEAR_VELOCITY_SCALE  # Z轴反转

        vel_rz = inputs['right_x'] * self.config.ANGULAR_VELOCITY_SCALE  # Yaw
        vel_ry = (inputs['lt'] - inputs['rt']) * self.config.ANGULAR_VELOCITY_SCALE  # Pitch
        vel_rx = (inputs['button_a'] - inputs['button_b']) * self.config.ANGULAR_VELOCITY_SCALE  # Roll

        # 保存速度（用于显示）
        self.state.target_velocity = [vel_x, vel_y, vel_z, vel_rx, vel_ry, vel_rz]

        # 计算位置增量
        target = [
            current[0] + vel_x * dt,
            current[1] + vel_y * dt,
            current[2] + vel_z * dt,
            current[3] + vel_rx * dt,
            current[4] + vel_ry * dt,
            current[5] + vel_rz * dt
        ]

        return target

    def send_control_command(self, target_pose: List[float]):
        """发送控制命令到机械臂"""
        try:
            # 转换单位：mm → 0.001mm, deg → 0.001deg
            X = int(target_pose[0] * 1000)
            Y = int(target_pose[1] * 1000)
            Z = int(target_pose[2] * 1000)
            RX = int(target_pose[3] * 1000)
            RY = int(target_pose[4] * 1000)
            RZ = int(target_pose[5] * 1000)

            # 计算速度百分比（根据当前速度动态调整）
            max_vel = max(abs(v) for v in self.state.target_velocity)
            if max_vel < 1.0:
                speed_percent = 10
            elif max_vel < 10.0:
                speed_percent = 30
            elif max_vel < 30.0:
                speed_percent = 60
            else:
                speed_percent = 100

            # 设置运动模式
            self.piper.MotionCtrl_2(0x01, 0x00, speed_percent, 0x00)

            # 发送末端位姿命令
            self.piper.EndPoseCtrl(X, Y, Z, RX, RY, RZ)

            self.safety.last_can_communication = time.time()
            self.safety.consecutive_errors = 0

        except Exception as e:
            print(f"[ERROR] 发送控制命令失败: {e}")
            self.safety.consecutive_errors += 1
            if self.safety.consecutive_errors > self.safety.max_consecutive_errors:
                self.safety.trigger_emergency_stop()

    def send_stop_command(self):
        """发送停止命令（保持当前位置）"""
        try:
            current = self.state.current_pose
            X = int(current[0] * 1000)
            Y = int(current[1] * 1000)
            Z = int(current[2] * 1000)
            RX = int(current[3] * 1000)
            RY = int(current[4] * 1000)
            RZ = int(current[5] * 1000)

            self.piper.MotionCtrl_2(0x01, 0x00, 10, 0x00)
            self.piper.EndPoseCtrl(X, Y, Z, RX, RY, RZ)

            # 清零速度
            self.state.target_velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        except Exception as e:
            print(f"[ERROR] 发送停止命令失败: {e}")

    def handle_gripper(self, inputs: dict):
        """处理夹爪控制"""
        try:
            if inputs['dpad_up']:
                # 张开夹爪
                if self.state.gripper_state != "opening":
                    self.state.gripper_state = "opening"
                    self.state.gripper_target = self.config.GRIPPER_OPEN_POS
                    self.piper.GripperCtrl(
                        self.config.GRIPPER_OPEN_POS,
                        self.config.GRIPPER_SPEED,
                        self.config.GRIPPER_FORCE,
                        0
                    )
            elif inputs['dpad_down']:
                # 闭合夹爪
                if self.state.gripper_state != "closing":
                    self.state.gripper_state = "closing"
                    self.state.gripper_target = self.config.GRIPPER_CLOSE_POS
                    self.piper.GripperCtrl(
                        self.config.GRIPPER_CLOSE_POS,
                        self.config.GRIPPER_SPEED,
                        self.config.GRIPPER_FORCE,
                        0
                    )
            else:
                # 没有按键，保持当前状态
                if self.state.gripper_state != "idle":
                    self.state.gripper_state = "idle"

        except Exception as e:
            print(f"[ERROR] 夹爪控制失败: {e}")

    def update_current_pose(self):
        """读取当前机械臂位姿"""
        try:
            end_pose = self.piper.GetArmEndPoseMsgs()
            self.state.current_pose = [
                end_pose.end_pose.X_axis / 1000.0,
                end_pose.end_pose.Y_axis / 1000.0,
                end_pose.end_pose.Z_axis / 1000.0,
                end_pose.end_pose.RX_axis / 1000.0,
                end_pose.end_pose.RY_axis / 1000.0,
                end_pose.end_pose.RZ_axis / 1000.0
            ]
        except Exception as e:
            print(f"[ERROR] 读取位姿失败: {e}")
            self.safety.consecutive_errors += 1

    def update_display(self):
        """更新状态显示"""
        pose = self.state.current_pose
        vel = self.state.target_velocity
        enabled_str = "已使能" if self.state.enabled else "未使能"
        gripper_str = self.state.gripper_state

        freq = self.config.CONTROL_FREQUENCY

        status = (f"[{enabled_str}] 频率:{freq}Hz | "
                 f"位姿:X={pose[0]:.0f}/Y={pose[1]:.0f}/Z={pose[2]:.0f}/"
                 f"RX={pose[3]:.0f}/RY={pose[4]:.0f}/RZ={pose[5]:.0f} | "
                 f"速度:vX={vel[0]:.0f}/vY={vel[1]:.0f}/vZ={vel[2]:.0f} | "
                 f"[夹爪:{gripper_str}]")

        # 使用\r返回行首，实现同行更新
        print(f"\r{status}", end='', flush=True)

    def run(self):
        """主控制循环"""
        clock = pygame.time.Clock()

        try:
            while not self.should_exit:
                loop_start = time.time()

                try:
                    # 读取控制器输入
                    inputs = self.read_controller_inputs()

                    # 检查使能键
                    lb_pressed = inputs['button_lb']

                    if lb_pressed:
                        if not self.state.enabled:
                            print("\n[INFO] 控制器已使能")
                            self.state.enabled = True

                        # 更新当前位姿
                        self.update_current_pose()

                        # 计算目标位姿
                        dt = time.time() - self.state.last_update_time
                        target_pose = self.calculate_target_pose(inputs, dt)

                        # 安全检查
                        is_safe, error_msg = self.safety.check_safety(
                            self.state.current_pose,
                            target_pose
                        )

                        if is_safe:
                            # 发送控制命令
                            self.send_control_command(target_pose)
                        else:
                            print(f"\n[SAFETY] {error_msg} - 停止运动")
                            self.send_stop_command()

                        # 处理夹爪
                        self.handle_gripper(inputs)

                    else:
                        if self.state.enabled:
                            print("\n[INFO] 控制器已失能 - 停止运动")
                            self.state.enabled = False

                        # LB未按下，发送停止命令
                        self.send_stop_command()

                    # 更新时间戳
                    self.state.last_update_time = time.time()
                    self.frame_count += 1

                    # 每10帧更新一次显示
                    if self.frame_count % 10 == 0:
                        self.update_display()

                except Exception as e:
                    print(f"\n[ERROR] 控制循环异常: {e}")
                    self.safety.consecutive_errors += 1
                    if self.safety.consecutive_errors > self.safety.max_consecutive_errors:
                        print("[CRITICAL] 连续错误过多，触发紧急停止")
                        self.safety.trigger_emergency_stop()
                        break

                # 控制循环频率
                clock.tick(self.config.CONTROL_FREQUENCY)

        except KeyboardInterrupt:
            print("\n\n[INFO] 用户请求退出...")
        except Exception as e:
            print(f"\n[CRITICAL] 严重错误: {e}")
        finally:
            self.shutdown()

    def shutdown(self):
        """安全关闭系统"""
        print("\n" + "="*60)
        print("正在安全关闭系统...")
        print("="*60)

        try:
            if self.piper is not None:
                # 停止运动
                print("[1/4] 停止机械臂运动...")
                self.send_stop_command()
                time.sleep(0.1)

                # 闭合夹爪
                print("[2/4] 闭合夹爪...")
                self.piper.GripperCtrl(0, 1000, 0x01, 0)
                time.sleep(0.2)

                # 失能机械臂
                print("[3/4] 失能机械臂...")
                self.piper.DisableArm(7)
                time.sleep(0.1)

                # 断开连接
                print("[4/4] 断开CAN连接...")

        except Exception as e:
            print(f"[WARNING] 关闭过程出现异常: {e}")

        finally:
            if self.joystick is not None:
                pygame.quit()

            print("="*60)
            print("系统已安全关闭")
            print("="*60)


def main():
    """主函数"""
    controller = XboxArmController("can_piper")

    if controller.initialize():
        controller.run()
    else:
        print("[FAILED] 系统初始化失败")
        sys.exit(1)


if __name__ == "__main__":
    main()
