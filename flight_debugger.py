import sys
import json
import socket
import time
def set_parameter(parameter_name, value):
    param_types = {
        "roll_kp": float,
        "roll_ki": float,
        "roll_kd": float,
        "pitch_kp": float,
        "pitch_ki": float,
        "pitch_kd": float,
        "yaw_kp": float,
        "yaw_ki": float,
        "yaw_kd": float,
        "throttle_kp": float,
        "throttle_ki": float,
        "throttle_kd": float,
        "target_roll": float,
        "target_pitch": float,
        "target_yaw": float,
        "target_throttle": float,
        "accel_noise": float,
        "gyro_noise": float,
        "simulate_wind": bool,
        "wind_strength": float,
        "wind_direction": float,
        "enable_gps": bool,
        "gps_accuracy": float,
        "simulate_failure": bool,
        "failure_type": str
    }
    if parameter_name not in param_types:
        return False, f"未知参数: {parameter_name}"
    try:
        if param_types[parameter_name] == bool:
            converted_value = value.lower() in ('true', 'yes', '1', 'y')
        else:
            converted_value = param_types[parameter_name](value)
        params = load_parameters()
        params[parameter_name] = converted_value
        save_parameters(params)
        broadcast_parameter_change(parameter_name, converted_value)
        return True, f"已设置 {parameter_name} = {converted_value}"
    except ValueError:
        return False, f"参数值类型错误: {parameter_name} 应为 {param_types[parameter_name].__name__}"
def load_parameters():
    try:
        with open('flight_params.json', 'r') as f:
            return json.load(f)
    except (FileNotFoundError, json.JSONDecodeError):
        default_params = {
            "roll_kp": 2.5,
            "roll_ki": 0.1,
            "roll_kd": 0.5,
            "pitch_kp": 2.5,
            "pitch_ki": 0.1,
            "pitch_kd": 0.5,
            "yaw_kp": 1.0,
            "yaw_ki": 0.05,
            "yaw_kd": 0.2,
            "throttle_kp": 5.0,
            "throttle_ki": 0.2,
            "throttle_kd": 1.0,
            "target_roll": 0.0,
            "target_pitch": 0.0,
            "target_yaw": 0.0,
            "target_throttle": 0.5,
            "accel_noise": 0.01,
            "gyro_noise": 0.01,
            "simulate_wind": False,
            "wind_strength": 0.1,
            "wind_direction": 0.0,
            "enable_gps": True,
            "gps_accuracy": 0.5,
            "simulate_failure": False,
            "failure_type": "none"
        }
        save_parameters(default_params)
        return default_params
def save_parameters(params):
    with open('flight_params.json', 'w') as f:
        json.dump(params, f, indent=2)
def broadcast_parameter_change(param_name, value):
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            message = json.dumps({"param": param_name, "value": value}).encode()
            s.sendto(message, ('<broadcast>', 5005))
    except Exception:
        pass
def list_parameters():
    params = load_parameters()
    param_info = {}
    for param, value in params.items():
        param_info[param] = str(value)
    return param_info
def interactive_mode():
    print("飞控调试器")
    print("输入 'help' 获取帮助")
    while True:
        cmd = input(">>> ").strip().lower()
        if not cmd:
            continue
        if cmd in ('exit', 'quit', 'q'):
            break
        elif cmd == 'help':
            print("可用命令:")
            print("  set <参数名> <值>  - 设置参数值")
            print("  list              - 列出所有参数及其当前值")
            print("  exit/quit/q       - 退出程序")
        elif cmd == 'list':
            params = list_parameters()
            for param, value in params.items():
                print(f"  {param}: {value}")
        elif cmd.startswith('set '):
            parts = cmd.split(maxsplit=2)
            if len(parts) < 3:
                print("用法: set <参数名> <值>")
                continue
            param_name = parts[1]
            value = parts[2]
            success, message = set_parameter(param_name, value)
            print(message)
        else:
            print(f"未知命令: {cmd}")
def main():
    if len(sys.argv) == 1:
        interactive_mode()
    elif len(sys.argv) == 3:
        _, param_name, value = sys.argv
        success, message = set_parameter(param_name, value)
        print(message)
    else:
        print("用法:")
        print("  python flight_debugger.py            - 启动交互式模式")
        print("  python flight_debugger.py <参数名> <值>  - 设置单个参数")
if __name__ == "__main__":
    main()

# x0r_fl0w