import tkinter as tk
from tkinter import ttk, LabelFrame, Scale
import math
import json
import socket
import threading
import time
import random
class FlightUISimulator:
    def __init__(self, root):
        self.root = root
        self.root.title("飞控可视化调试器")
        self.root.geometry("1000x700")
        self.root.configure(bg="#f0f0f0")
        self.current_roll = 0.0
        self.current_pitch = 0.0
        self.current_yaw = 0.0
        self.target_roll = 0.0
        self.target_pitch = 0.0
        self.target_yaw = 0.0
        self.target_throttle = 0.5
        self.motor_values = [0.5, 0.5, 0.5, 0.5]
        self.sensor_values = {"accel": [0,0,9.8], "gyro": [0,0,0], "baro": 1013.25}
        self.params = {}
        self.param_lock = threading.Lock()
        self.running = True
        self.setup_ui()
        self.start_udp_server()
        self.start_simulation_thread()
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
    def setup_ui(self):
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.pack(fill=tk.BOTH, expand=True)
        left_frame = ttk.Frame(main_frame, padding="5")
        left_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        right_frame = ttk.Frame(main_frame, padding="5", width=300)
        right_frame.pack(side=tk.RIGHT, fill=tk.BOTH)
        self.attitude_frame = LabelFrame(left_frame, text="飞机姿态", padx=10, pady=10)
        self.attitude_frame.pack(fill=tk.BOTH, expand=True, pady=(0, 10))
        self.canvas_width = 400
        self.canvas_height = 400
        self.attitude_canvas = tk.Canvas(self.attitude_frame, width=self.canvas_width, height=self.canvas_height, bg="#000022")
        self.attitude_canvas.pack(fill=tk.BOTH, expand=True)
        self.info_frame = LabelFrame(left_frame, text="飞行数据", padx=10, pady=10)
        self.info_frame.pack(fill=tk.BOTH, expand=True)
        info_grid_frame = ttk.Frame(self.info_frame)
        info_grid_frame.pack(fill=tk.BOTH, expand=True)
        labels = [
            ("当前滚转: ", "roll_value"),
            ("当前俯仰: ", "pitch_value"),
            ("当前偏航: ", "yaw_value"),
            ("目标滚转: ", "target_roll_value"),
            ("目标俯仰: ", "target_pitch_value"),
            ("目标偏航: ", "target_yaw_value"),
            ("目标油门: ", "throttle_value")
        ]
        self.info_labels = {}
        for i, (text, key) in enumerate(labels):
            ttk.Label(info_grid_frame, text=text).grid(row=i, column=0, sticky=tk.W, padx=5, pady=2)
            self.info_labels[key] = ttk.Label(info_grid_frame, text="0.0°")
            self.info_labels[key].grid(row=i, column=1, sticky=tk.W, padx=5, pady=2)
        sensor_labels = [
            ("加速度X: ", "accel_x"),
            ("加速度Y: ", "accel_y"),
            ("加速度Z: ", "accel_z"),
            ("陀螺仪X: ", "gyro_x"),
            ("陀螺仪Y: ", "gyro_y"),
            ("陀螺仪Z: ", "gyro_z"),
            ("气压计: ", "baro")
        ]
        for i, (text, key) in enumerate(sensor_labels):
            ttk.Label(info_grid_frame, text=text).grid(row=i, column=2, sticky=tk.W, padx=5, pady=2)
            self.info_labels[key] = ttk.Label(info_grid_frame, text="0.0")
            self.info_labels[key].grid(row=i, column=3, sticky=tk.W, padx=5, pady=2)
        self.motor_frame = LabelFrame(right_frame, text="电机输出", padx=10, pady=10)
        self.motor_frame.pack(fill=tk.BOTH, expand=True, pady=(0, 10))
        self.motor_bars = []
        motor_labels = ["电机1", "电机2", "电机3", "电机4"]
        for i, label in enumerate(motor_labels):
            ttk.Label(self.motor_frame, text=label).pack(anchor=tk.W, padx=5, pady=(5, 2))
            bar_frame = ttk.Frame(self.motor_frame)
            bar_frame.pack(fill=tk.X, padx=5, pady=2)
            bar = ttk.Progressbar(bar_frame, orient=tk.HORIZONTAL, length=100, mode='determinate')
            bar.pack(side=tk.LEFT, fill=tk.X, expand=True)
            bar_label = ttk.Label(bar_frame, text="0%")
            bar_label.pack(side=tk.RIGHT, padx=5)
            self.motor_bars.append((bar, bar_label))
        self.control_frame = LabelFrame(right_frame, text="参数控制", padx=10, pady=10)
        self.control_frame.pack(fill=tk.BOTH, expand=True)
        self.controls = {
            "target_roll": self.create_scale(self.control_frame, "目标滚转", -45, 45, 0, self.update_target_roll),
            "target_pitch": self.create_scale(self.control_frame, "目标俯仰", -45, 45, 0, self.update_target_pitch),
            "target_yaw": self.create_scale(self.control_frame, "目标偏航", -180, 180, 0, self.update_target_yaw),
            "target_throttle": self.create_scale(self.control_frame, "目标油门", 0, 100, 50, self.update_target_throttle),
            "roll_kp": self.create_scale(self.control_frame, "Roll Kp", 0, 5, 2.5, self.update_param, step=0.1),
            "pitch_kp": self.create_scale(self.control_frame, "Pitch Kp", 0, 5, 2.5, self.update_param, step=0.1),
            "yaw_kp": self.create_scale(self.control_frame, "Yaw Kp", 0, 5, 1.0, self.update_param, step=0.1),
        }
    def create_scale(self, parent, label, from_, to, default, command, step=1):
        frame = ttk.Frame(parent)
        frame.pack(fill=tk.X, padx=5, pady=5)
        ttk.Label(frame, text=label).pack(anchor=tk.W)
        value_var = tk.DoubleVar(value=default)
        scale = Scale(frame, from_=from_, to=to, orient=tk.HORIZONTAL, variable=value_var, command=command, resolution=step, length=200)
        scale.pack(fill=tk.X, side=tk.LEFT, padx=(0, 5))
        value_label = ttk.Label(frame, text=f"{default}")
        value_label.pack(side=tk.RIGHT)
        return (scale, value_var, value_label)
    def update_target_roll(self, value):
        self.target_roll = float(value)
        self.controls["target_roll"][2].config(text=f"{self.target_roll:.1f}")
        self.send_parameter("target_roll", self.target_roll)
    def update_target_pitch(self, value):
        self.target_pitch = float(value)
        self.controls["target_pitch"][2].config(text=f"{self.target_pitch:.1f}")
        self.send_parameter("target_pitch", self.target_pitch)
    def update_target_yaw(self, value):
        self.target_yaw = float(value)
        self.controls["target_yaw"][2].config(text=f"{self.target_yaw:.1f}")
        self.send_parameter("target_yaw", self.target_yaw)
    def update_target_throttle(self, value):
        self.target_throttle = float(value) / 100.0
        self.controls["target_throttle"][2].config(text=f"{self.target_throttle:.2f}")
        self.send_parameter("target_throttle", self.target_throttle)
    def update_param(self, value):
        widget = self.root.focus_get()
        if widget:
            for key, (scale, var, label) in self.controls.items():
                if scale == widget:
                    val = var.get()
                    label.config(text=f"{val:.2f}")
                    self.send_parameter(key, val)
                    break
    def send_parameter(self, param_name, value):
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
                s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
                message = json.dumps({"param": param_name, "value": value}).encode()
                s.sendto(message, ('<broadcast>', 5005))
        except Exception:
            pass
    def start_udp_server(self):
        def udp_listener():
            try:
                with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
                    s.bind(('0.0.0.0', 5006))
                    s.settimeout(1.0)
                    while self.running:
                        try:
                            data, _ = s.recvfrom(4096)
                            try:
                                message = json.loads(data.decode())
                                with self.param_lock:
                                    if 'roll' in message:
                                        self.current_roll = message['roll']
                                    if 'pitch' in message:
                                        self.current_pitch = message['pitch']
                                    if 'yaw' in message:
                                        self.current_yaw = message['yaw']
                                    if 'motors' in message:
                                        self.motor_values = message['motors']
                                    if 'sensors' in message:
                                        self.sensor_values = message['sensors']
                            except json.JSONDecodeError:
                                pass
                        except socket.timeout:
                            continue
            except Exception:
                pass
        self.udp_thread = threading.Thread(target=udp_listener, daemon=True)
        self.udp_thread.start()
    def start_simulation_thread(self):
        def simulation_loop():
            while self.running:
                self.simulate_data()
                self.root.after(0, self.update_ui)
                time.sleep(0.05)
        self.simulation_thread = threading.Thread(target=simulation_loop, daemon=True)
        self.simulation_thread.start()
    def simulate_data(self):
        dt = 0.05
        kp = 2.5
        ki = 0.1
        kd = 0.5
        roll_error = self.target_roll - self.current_roll
        pitch_error = self.target_pitch - self.current_pitch
        yaw_error = self.target_yaw - self.current_yaw
        self.current_roll += roll_error * kp * dt + random.gauss(0, 0.1)
        self.current_pitch += pitch_error * kp * dt + random.gauss(0, 0.1)
        self.current_yaw += yaw_error * 0.5 * dt + random.gauss(0, 0.2)
        self.current_roll = max(-90, min(90, self.current_roll))
        self.current_pitch = max(-90, min(90, self.current_pitch))
        if self.current_yaw > 180: self.current_yaw -= 360
        if self.current_yaw < -180: self.current_yaw += 360
        self.sensor_values["accel"][0] = -math.sin(math.radians(self.current_pitch)) * 9.8 + random.gauss(0, 0.1)
        self.sensor_values["accel"][1] = math.sin(math.radians(self.current_roll)) * math.cos(math.radians(self.current_pitch)) * 9.8 + random.gauss(0, 0.1)
        self.sensor_values["accel"][2] = math.cos(math.radians(self.current_roll)) * math.cos(math.radians(self.current_pitch)) * 9.8 + random.gauss(0, 0.1)
        self.sensor_values["gyro"][0] = (self.current_roll - getattr(self, 'prev_roll', 0)) / dt + random.gauss(0, 0.2)
        self.sensor_values["gyro"][1] = (self.current_pitch - getattr(self, 'prev_pitch', 0)) / dt + random.gauss(0, 0.2)
        self.sensor_values["gyro"][2] = (self.current_yaw - getattr(self, 'prev_yaw', 0)) / dt + random.gauss(0, 0.5)
        self.prev_roll = self.current_roll
        self.prev_pitch = self.current_pitch
        self.prev_yaw = self.current_yaw
        self.motor_values[0] = self.target_throttle + (self.current_roll * 0.02) + (self.current_pitch * 0.02)
        self.motor_values[1] = self.target_throttle - (self.current_roll * 0.02) + (self.current_pitch * 0.02)
        self.motor_values[2] = self.target_throttle - (self.current_roll * 0.02) - (self.current_pitch * 0.02)
        self.motor_values[3] = self.target_throttle + (self.current_roll * 0.02) - (self.current_pitch * 0.02)
        self.motor_values = [max(0, min(1, m)) for m in self.motor_values]
    def update_ui(self):
        self.update_attitude_display()
        self.info_labels["roll_value"].config(text=f"{self.current_roll:.1f}°")
        self.info_labels["pitch_value"].config(text=f"{self.current_pitch:.1f}°")
        self.info_labels["yaw_value"].config(text=f"{self.current_yaw:.1f}°")
        self.info_labels["target_roll_value"].config(text=f"{self.target_roll:.1f}°")
        self.info_labels["target_pitch_value"].config(text=f"{self.target_pitch:.1f}°")
        self.info_labels["target_yaw_value"].config(text=f"{self.target_yaw:.1f}°")
        self.info_labels["throttle_value"].config(text=f"{self.target_throttle:.2f}")
        self.info_labels["accel_x"].config(text=f"{self.sensor_values['accel'][0]:.2f}")
        self.info_labels["accel_y"].config(text=f"{self.sensor_values['accel'][1]:.2f}")
        self.info_labels["accel_z"].config(text=f"{self.sensor_values['accel'][2]:.2f}")
        self.info_labels["gyro_x"].config(text=f"{self.sensor_values['gyro'][0]:.2f}")
        self.info_labels["gyro_y"].config(text=f"{self.sensor_values['gyro'][1]:.2f}")
        self.info_labels["gyro_z"].config(text=f"{self.sensor_values['gyro'][2]:.2f}")
        self.info_labels["baro"].config(text=f"{self.sensor_values['baro']:.2f}")
        for i, (bar, label) in enumerate(self.motor_bars):
            value = self.motor_values[i] * 100
            bar['value'] = value
            label.config(text=f"{value:.0f}%")
    def update_attitude_display(self):
        self.attitude_canvas.delete("all")
        width = self.attitude_canvas.winfo_width()
        height = self.attitude_canvas.winfo_height()
        if width == 1 or height == 1:
            width, height = self.canvas_width, self.canvas_height
        center_x = width // 2
        center_y = height // 2
        radius = min(width, height) // 2 - 20
        roll_rad = math.radians(self.current_roll)
        pitch_rad = math.radians(self.current_pitch)
        self.attitude_canvas.create_oval(center_x - radius, center_y - radius, center_x + radius, center_y + radius, outline="#444466", fill="#000022")
        for i in range(-90, 91, 15):
            angle_rad = math.radians(i)
            x1 = center_x + radius * math.cos(angle_rad)
            y1 = center_y + radius * math.sin(angle_rad)
            x2 = center_x + (radius - 10) * math.cos(angle_rad)
            y2 = center_y + (radius - 10) * math.sin(angle_rad)
            self.attitude_canvas.create_line(x1, y1, x2, y2, fill="#444466")
            if i % 30 == 0:
                text_x = center_x + (radius - 20) * math.cos(angle_rad)
                text_y = center_y + (radius - 20) * math.sin(angle_rad)
                self.attitude_canvas.create_text(text_x, text_y, text=str(i), fill="#444466")
        for i in range(-90, 91, 15):
            angle_rad = math.radians(i)
            x1 = center_x + radius * math.cos(angle_rad + math.pi/2)
            y1 = center_y + radius * math.sin(angle_rad + math.pi/2)
            x2 = center_x + (radius - 10) * math.cos(angle_rad + math.pi/2)
            y2 = center_y + (radius - 10) * math.sin(angle_rad + math.pi/2)
            self.attitude_canvas.create_line(x1, y1, x2, y2, fill="#444466")
        plane_size = radius * 0.4
        rot_matrix = self.get_rotation_matrix(roll_rad, pitch_rad, 0)
        points = [
            (0, -plane_size/2, 0),
            (plane_size/2, plane_size/2, 0),
            (-plane_size/2, plane_size/2, 0)
        ]
        projected_points = []
        for x, y, z in points:
            x_proj, y_proj, _ = self.apply_rotation(x, y, z, rot_matrix)
            projected_points.append((center_x + x_proj, center_y + y_proj))
        self.attitude_canvas.create_polygon(projected_points, fill="#FF4444", outline="#FF0000")
        wing_length = plane_size * 0.8
        tail_length = plane_size * 0.4
        wing_points = [
            (center_x - wing_length/2, center_y),
            (center_x + wing_length/2, center_y),
            (center_x + wing_length/2, center_y + 5),
            (center_x - wing_length/2, center_y + 5)
        ]
        tail_points = [
            (center_x, center_y - plane_size/2),
            (center_x, center_y - plane_size/2 - tail_length),
            (center_x + 5, center_y - plane_size/2 - tail_length),
            (center_x + 5, center_y - plane_size/2)
        ]
        wing_rot_matrix = self.get_rotation_matrix(roll_rad, 0, 0)
        tail_rot_matrix = self.get_rotation_matrix(0, pitch_rad, 0)
        rotated_wing_points = []
        for x, y in wing_points:
            rel_x = x - center_x
            rel_y = y - center_y
            rx, ry, _ = self.apply_rotation(rel_x, rel_y, 0, wing_rot_matrix)
            rotated_wing_points.append((center_x + rx, center_y + ry))
        rotated_tail_points = []
        for x, y in tail_points:
            rel_x = x - center_x
            rel_y = y - center_y
            rx, ry, _ = self.apply_rotation(rel_x, rel_y, 0, tail_rot_matrix)
            rotated_tail_points.append((center_x + rx, center_y + ry))
        self.attitude_canvas.create_polygon(rotated_wing_points, fill="#4444FF", outline="#0000FF")
        self.attitude_canvas.create_polygon(rotated_tail_points, fill="#44FF44", outline="#00FF00")
    def get_rotation_matrix(self, roll, pitch, yaw):
        cr, sr = math.cos(roll), math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cy, sy = math.cos(yaw), math.sin(yaw)
        return [
            [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
            [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
            [-sp, cp*sr, cp*cr]
        ]
    def apply_rotation(self, x, y, z, rot_matrix):
        nx = x * rot_matrix[0][0] + y * rot_matrix[0][1] + z * rot_matrix[0][2]
        ny = x * rot_matrix[1][0] + y * rot_matrix[1][1] + z * rot_matrix[1][2]
        nz = x * rot_matrix[2][0] + y * rot_matrix[2][1] + z * rot_matrix[2][2]
        return (nx, ny, nz)
    def on_closing(self):
        self.running = False
        self.root.destroy()
if __name__ == "__main__":
    root = tk.Tk()
    app = FlightUISimulator(root)
    root.mainloop()

# x0r_fl0w