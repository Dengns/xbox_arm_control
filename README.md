# Xbox控制器控制Piper机械臂

使用Xbox控制器实时控制Piper机械臂的末端位姿（速度控制模式）。

## 功能特点

- **速度控制模式**：摇杆控制运动速度，松开后机械臂停止
- **安全使能开关**：必须按住LB键才能发送控制指令
- **完整6自由度控制**：支持XYZ平移和RX/RY/RZ旋转
- **夹爪控制**：使用十字键控制夹爪开合
- **多重安全保护**：位置限位、速度限制、通信监控、异常处理

## 硬件要求

- Piper机械臂（通过CAN总线连接）
- Xbox控制器（有线或无线）
- 运行Linux系统的计算机

## 软件依赖

```bash
pip install -r requirements.txt
```

或手动安装：

```bash
pip install pygame piper-sdk
```

## 使用说明

### 1. 启动程序

```bash
cd /home/qzl/Main/piper/xbox_arm_control
python3 xbox_arm_controller.py
```

### 2. 控制器按键映射

| 控制 | 功能 |
|------|------|
| **LB键（左肩）** | 使能开关（必须按住才能控制） |
| **左摇杆 X/Y** | 末端位置 X/Y 速度控制 |
| **右摇杆 Y** | 末端位置 Z 速度控制 |
| **右摇杆 X** | RZ 旋转速度（偏航/Yaw） |
| **LT 扳机** | RY 正向俯仰速度（Pitch+） |
| **RT 扳机** | RY 负向俯仰速度（Pitch-） |
| **A 按钮** | RX 正向翻滚速度（Roll+） |
| **B 按钮** | RX 负向翻滚速度（Roll-） |
| **十字键上** | 张开夹爪 |
| **十字键下** | 闭合夹爪 |

### 3. 操作流程

1. 确保Xbox控制器已连接
2. 确保Piper机械臂已通电并连接到CAN总线（can_piper接口）
3. 运行程序
4. 按住LB键使能控制
5. 使用摇杆和按钮控制机械臂
6. 松开LB键或摇杆，机械臂停止运动
7. 按Ctrl+C安全退出

### 4. 状态显示

程序运行时会实时显示：

```
[已使能] 频率:50Hz | 位姿:X=62/Y=-15/Z=216/RX=2/RY=88/RZ=-5 | 速度:vX=12/vY=-8/vZ=0 | [夹爪:idle]
```

包含信息：
- 使能状态
- 控制循环频率
- 当前位姿（mm和度）
- 目标速度（mm/s和deg/s）
- 夹爪状态

## 配置参数

可以在 `xbox_arm_controller.py` 中的 `ControlConfig` 类修改以下参数：

```python
# 速度缩放因子
LINEAR_VELOCITY_SCALE = 50.0   # mm/s (摇杆满偏时的速度)
ANGULAR_VELOCITY_SCALE = 30.0  # deg/s

# 速度限制
MAX_LINEAR_VELOCITY = 100.0
MAX_ANGULAR_VELOCITY = 60.0

# 死区设置
JOYSTICK_DEADZONE = 0.1  # 10%
TRIGGER_DEADZONE = 0.05  # 5%

# 位置限位（安全范围）
POSITION_LIMITS = {
    'X': (0, 400),        # mm
    'Y': (-300, 300),
    'Z': (0, 500),
    'RX': (-120, 120),    # degrees
    'RY': (-70, 70),
    'RZ': (-150, 150)
}

# 夹爪参数
GRIPPER_OPEN_POS = 50000     # 50mm
GRIPPER_CLOSE_POS = 0
GRIPPER_SPEED = 1000
GRIPPER_FORCE = 500

# 控制频率
CONTROL_FREQUENCY = 50   # Hz
```

### 参数调整建议

- **初学者**：降低速度至 30mm/s 和 20deg/s
- **熟练操作者**：提高至 70mm/s 和 45deg/s
- **精密操作**：降低至 20mm/s 和 10deg/s

## 安全机制

1. **使能开关**：LB键必须按住才能控制
2. **位置限位**：超出安全范围立即停止
3. **速度限制**：防止过快运动
4. **通信监控**：1秒无响应触发告警
5. **紧急停止**：Ctrl+C安全退出
6. **异常处理**：连续5次错误触发停止
7. **软限位警告**：接近边界时显示警告

## 故障排查

### 检测不到Xbox控制器

- 检查控制器连接：`ls /dev/input/js*`
- 尝试有线连接
- 检查pygame版本：`pip show pygame`

### CAN通信失败

```bash
# 检查CAN接口
ip link show can_piper

# 启动CAN接口（如果需要）
sudo ip link set can_piper up type can bitrate 1000000

# 测试CAN通信
candump can_piper
```

### 机械臂使能失败

- 检查机械臂电源
- 检查急停按钮是否释放
- 运行SDK测试程序验证连接

### 运动不流畅

- 降低速度缩放因子
- 增大死区值至0.15
- 检查CAN总线负载
- 降低控制频率至30Hz

## 技术细节

### 控制模式

使用**速度控制模式**：
- SDK的`EndPoseCtrl()`接收位置命令
- 在控制循环中计算：`target_pose = current_pose + velocity * dt`
- 每20ms（50Hz）发送一次更新的目标位置
- 松开摇杆时发送当前位置，实现停止

### 坐标系说明

- **X轴**：前后方向（0-400mm）
- **Y轴**：左右方向（-300到300mm）
- **Z轴**：上下方向（0-500mm）
- **RX轴**：绕X轴旋转/翻滚（-120到120度）
- **RY轴**：绕Y轴旋转/俯仰（-70到70度）
- **RZ轴**：绕Z轴旋转/偏航（-150到150度）

## 作者

基于Piper SDK开发

## 许可

与Piper SDK相同
