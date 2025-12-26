#!/bin/bash

echo "====================================================="
echo "   Piper机械臂 Xbox控制器启动脚本"
echo "====================================================="

# 1. 初始化CAN接口
echo "[1/2] 正在初始化CAN接口 (can_piper)..."
# bash /home/qzl/Main/piper/piper_sdk_demo/can_activate.sh can_piper 1000000
# bash can_activate.sh can_piper 1000000 "1-1.3:1.0"

if [ $? -ne 0 ]; then
    echo ""
    echo "[ERROR] CAN接口初始化失败！"
    echo "请检查："
    echo "  1. 机械臂是否已通电"
    echo "  2. CAN转USB设备是否已连接"
    echo "  3. 是否有sudo权限"
    echo ""
    exit 1
fi

echo ""
echo "[2/2] 启动Xbox控制程序..."
echo ""

# 2. 启动控制程序
cd /home/qzl/Main/piper/xbox_arm_control
python3 xbox_arm_controller.py
