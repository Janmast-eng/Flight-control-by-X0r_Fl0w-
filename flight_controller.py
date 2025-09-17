import math
import json
import socket
import threading
import random
sensor_data = {"accelerometer": [0.0, 0.0, 0.0], "gyroscope": [0.0, 0.0, 0.0], "barometer": 1013.25, "gps": [0.0, 0.0, 0.0]}
setpoint = {"roll": 0.0, "pitch": 0.0, "yaw": 0.0, "throttle": 0.0}
params = {}
param_lock = threading.Lock()
class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0.0
        self.integral = 0.0
    def compute(self, setpoint, process_value, dt):
        error = setpoint - process_value
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return output
    def update_params(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd

roll_pid = PIDController(2.5, 0.1, 0.5)
pitch_pid = PIDController(2.5, 0.1, 0.5)
yaw_pid = PIDController(1.0, 0.05, 0.2)
throttle_pid = PIDController(5.0, 0.2, 1.0)

def load_params():
    global params
    try:
        with open('flight_params.json', 'r') as f:
            with param_lock:
                params = json.load(f)
    except (FileNotFoundError, json.JSONDecodeError):
        pass

def start_param_server():
    def param_listener():
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.bind(('0.0.0.0', 5005))
            while True:
                data, _ = s.recvfrom(1024)
                try:
                    msg = json.loads(data.decode())
                    param_name = msg.get('param')
                    value = msg.get('value')
                    if param_name:
                        with param_lock:
                            params[param_name] = value
                except json.JSONDecodeError:
                    pass
    thread = threading.Thread(target=param_listener, daemon=True)
    thread.start()

load_params()
start_param_server()
def read_sensors():
    with param_lock:
        current_params = params.copy()
    accel_noise = current_params.get('accel_noise', 0.01)
    gyro_noise = current_params.get('gyro_noise', 0.01)
    simulate_wind = current_params.get('simulate_wind', False)
    
    accel = [
        random.gauss(0, accel_noise),
        random.gauss(0, accel_noise),
        9.8 + random.gauss(0, accel_noise)
    ]
    
    gyro = [
        random.gauss(0, gyro_noise),
        random.gauss(0, gyro_noise),
        random.gauss(0, gyro_noise)
    ]
    
    if simulate_wind:
        wind_strength = current_params.get('wind_strength', 0.1)
        wind_dir = current_params.get('wind_direction', 0.0)
        accel[0] += wind_strength * math.cos(wind_dir)
        accel[1] += wind_strength * math.sin(wind_dir)
    
    return {
        "accelerometer": accel,
        "gyroscope": gyro,
        "barometer": 1013.25 + random.gauss(0, 0.1),
        "gps": [0.0, 0.0, 0.0] if not current_params.get('enable_gps', True) else [
            random.gauss(0, current_params.get('gps_accuracy', 0.5)),
            random.gauss(0, current_params.get('gps_accuracy', 0.5)),
            random.gauss(0, current_params.get('gps_accuracy', 0.5))
        ]
    }
def calculate_attitude(accel, gyro, dt):
    ax, ay, az = accel
    gx, gy, gz = gyro
    roll = math.atan2(ay, math.sqrt(ax**2 + az**2)) * 180 / math.pi
    pitch = math.atan2(-ax, math.sqrt(ay**2 + az**2)) * 180 / math.pi
    return roll, pitch, 0.0
def send_motor_commands(motors):
    pass

def send_flight_data_to_ui(roll, pitch, yaw, motors, sensors):
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            message = json.dumps({
                "roll": roll,
                "pitch": pitch,
                "yaw": yaw,
                "motors": motors,
                "sensors": {
                    "accel": sensors["accelerometer"],
                    "gyro": sensors["gyroscope"],
                    "baro": sensors["barometer"]
                }
            }).encode()
            s.sendto(message, ('localhost', 5006))
    except Exception:
        pass
def update_setpoint(new_setpoint):
    setpoint.update(new_setpoint)
def main():
    dt = 0.01
    last_param_update = time.time()
    while True:
        current_time = time.time()
        if current_time - last_param_update > 1.0:
            with param_lock:
                current_params = params.copy()
            roll_pid.update_params(
                current_params.get('roll_kp', 2.5),
                current_params.get('roll_ki', 0.1),
                current_params.get('roll_kd', 0.5)
            )
            pitch_pid.update_params(
                current_params.get('pitch_kp', 2.5),
                current_params.get('pitch_ki', 0.1),
                current_params.get('pitch_kd', 0.5)
            )
            yaw_pid.update_params(
                current_params.get('yaw_kp', 1.0),
                current_params.get('yaw_ki', 0.05),
                current_params.get('yaw_kd', 0.2)
            )
            throttle_pid.update_params(
                current_params.get('throttle_kp', 5.0),
                current_params.get('throttle_ki', 0.2),
                current_params.get('throttle_kd', 1.0)
            )
            setpoint["roll"] = current_params.get('target_roll', 0.0)
            setpoint["pitch"] = current_params.get('target_pitch', 0.0)
            setpoint["yaw"] = current_params.get('target_yaw', 0.0)
            setpoint["throttle"] = current_params.get('target_throttle', 0.5)
            last_param_update = current_time
        
        sensors = read_sensors()
        accel = sensors["accelerometer"]
        gyro = sensors["gyroscope"]
        current_roll, current_pitch, current_yaw = calculate_attitude(accel, gyro, dt)
        
        with param_lock:
            simulate_failure = params.get('simulate_failure', False)
            failure_type = params.get('failure_type', 'none')
        
        if simulate_failure and failure_type != 'none':
            if failure_type == 'motor1':
                motor1 = 0.0
                motor2 = motor4 = 0.7
                motor3 = 0.3
            elif failure_type == 'motor2':
                motor2 = 0.0
                motor1 = motor3 = 0.7
                motor4 = 0.3
            elif failure_type == 'motor3':
                motor3 = 0.0
                motor2 = motor4 = 0.7
                motor1 = 0.3
            elif failure_type == 'motor4':
                motor4 = 0.0
                motor1 = motor3 = 0.7
                motor2 = 0.3
            elif failure_type == 'sensor':
                current_roll += 10.0 * random.random()
                current_pitch += 10.0 * random.random()
        else:
            roll_output = roll_pid.compute(setpoint["roll"], current_roll, dt)
            pitch_output = pitch_pid.compute(setpoint["pitch"], current_pitch, dt)
            yaw_output = yaw_pid.compute(setpoint["yaw"], current_yaw, dt)
            throttle_output = setpoint["throttle"]
            motor1 = throttle_output + roll_output + pitch_output - yaw_output
            motor2 = throttle_output - roll_output + pitch_output + yaw_output
            motor3 = throttle_output - roll_output - pitch_output - yaw_output
            motor4 = throttle_output + roll_output - pitch_output + yaw_output
        
        motors = [motor1, motor2, motor3, motor4]
        motors = [max(0, min(1, m)) for m in motors]
        send_motor_commands(motors)
        send_flight_data_to_ui(current_roll, current_pitch, current_yaw, motors, sensors)
        time.sleep(dt)
if __name__ == "__main__":
    import time
    main()

# x0r_fl0w