   # 飞控系统

这是一个基于Python的简单飞控系统，包含飞控主程序、命令行调试工具和带UI的可视化调试器。

## 文件结构

- `flight_controller.py` - 飞控主程序，实现传感器读取、姿态计算、PID控制和电机指令输出
- `flight_debugger.py` - 命令行调试工具，用于手动设置各种环境参数
- `flight_ui_debugger.py` - 带图形界面的可视化调试器，显示飞机状态和各种角度
- `tkinter_test.py` - Tkinter库测试脚本，验证GUI功能是否正常

## 运行方法

1. 运行飞控主程序：
   ```
   python flight_controller.py
   ```

2. 运行命令行调试器：
   ```
   python flight_debugger.py
   ```

3. 运行可视化UI调试器：
   ```
   python flight_ui_debugger.py
   ```

## 功能说明

### 飞控主程序 (flight_controller.py)
- 实现PID控制算法
- 模拟传感器数据读取
- 计算飞机姿态角
- 生成电机控制指令
- 支持与调试器通信，接收参数更新
- 向UI调试器发送飞行状态数据

### 命令行调试器 (flight_debugger.py)
- 支持交互式模式和命令行参数设置
- 可配置PID参数、目标姿态、传感器噪声等
- 通过UDP广播参数变更
- 支持参数保存和加载

### 可视化UI调试器 (flight_ui_debugger.py)
- 使用Tkinter实现图形界面
- 显示飞机姿态可视化（通过Canvas绘制）
- 显示飞行数据和电机输出
- 提供参数控制滑块调整目标姿态和PID参数
- 通过UDP与飞控程序通信

## 系统要求

- Python 3.x
- 标准库：math, json, socket, threading, time, random
- UI调试器需要：tkinter

# x0r_fl0w