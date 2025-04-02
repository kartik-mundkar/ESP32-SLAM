import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import asyncio
import websockets
import json
import threading
import time

ESP32_WS_URL = "ws://192.168.137.120:81"

# IMU Offsets
gyro_offset = 0
accelX_offset = 0
accelY_offset = 0

# Constants
g = 9.81  # Gravity
dt = 0.05  # 20Hz update rate
ALPHA_GYRO = 0.995  # Trust more on gyro, but correct drift
ACCEL_ALPHA = 0.3  # Stronger smoothing for acceleration
VEL_DECAY = 0.96  # Reduce oscillations by velocity decay
BIAS_CORRECTION_RATE = 0.001  # Slowly adjust gyro bias
STATIONARY_THRESHOLD = 0.04  # Threshold for zero-velocity update (ZUPT)

# State Variables
X, Y = 0.0, 0.0  # Global position
Vx, Vy = 0.0, 0.0  # Global velocity
yaw = 0.0  # Orientation
gyroZ_filtered = 0.0  # Filtered gyro
ax_m, ay_m = 0.0, 0.0  # Filtered acceleration
gyro_bias = 0.0  # Bias correction for gyroscope
last_rest_time = time.time()

# Data storage for plotting
x_data, y_data = [0], [0]

def kalman_correct(estimate, measurement, Q=1e-4, R=1e-2):
    P = 1.0  # Estimation variance
    K = P / (P + R)
    return estimate + K * (measurement - estimate)

def process_imu_data(accelX, accelY, accelZ, gyroZ):
    global X, Y, Vx, Vy, yaw, gyroZ_filtered, gyro_bias, last_rest_time, ax_m, ay_m  
    gyroZ -= gyro_offset
    accelX -= accelX_offset
    accelY -= accelY_offset
    
    gyro_bias = (1 - BIAS_CORRECTION_RATE) * gyro_bias + BIAS_CORRECTION_RATE * gyroZ
    gyroZ_filtered = ALPHA_GYRO * (gyroZ_filtered + (gyroZ - gyro_bias) * dt) + (1 - ALPHA_GYRO) * gyroZ
    yaw = 0.99 * (yaw + gyroZ_filtered * dt) + 0.01 * np.arctan2(accelY, accelX)
    
    ax_m = (1 - ACCEL_ALPHA) * ax_m + ACCEL_ALPHA * (accelX * g)  
    ay_m = (1 - ACCEL_ALPHA) * ay_m + ACCEL_ALPHA * (accelY * g)
    
    ax_global = ax_m * np.cos(yaw) - ay_m * np.sin(yaw)
    ay_global = ax_m * np.sin(yaw) + ay_m * np.cos(yaw)
    
    if abs(ax_global) < STATIONARY_THRESHOLD and abs(ay_global) < STATIONARY_THRESHOLD:
        if time.time() - last_rest_time > 0.5:
            Vx, Vy = 0, 0  
        return
    else:
        last_rest_time = time.time()
    
    Vx = kalman_correct(Vx * VEL_DECAY, Vx + ax_global * dt)
    Vy = kalman_correct(Vy * VEL_DECAY, Vy + ay_global * dt)
    
    X += Vx * dt
    Y += Vy * dt
    
    x_data.append(X)
    y_data.append(Y)

# Setup Matplotlib
fig, ax = plt.subplots(figsize=(8, 6))
ax.set_title("Car Path Tracking (Global Frame)")
ax.set_xlabel("X Position (m)")
ax.set_ylabel("Y Position (m)")
ax.grid(True)
line_path, = ax.plot([], [], 'bo-', label="Car Path")
ax.legend()


def update_plot(frame):
    """ Update path plot dynamically with auto zoom """
    line_path.set_data(x_data, y_data)
    
    if len(x_data) > 10:
        x_min, x_max = min(x_data), max(x_data)
        y_min, y_max = min(y_data), max(y_data)
        padding = 1  # Add padding for better view
        ax.set_xlim(x_min - padding, x_max + padding)
        ax.set_ylim(y_min - padding, y_max + padding)
    
    return line_path,


async def receive_data():
    """ WebSocket Client to receive IMU data """
    async with websockets.connect(ESP32_WS_URL) as websocket:
        print("Connected to ESP32 WebSocket!")
        await websocket.recv()
        while True:
            try:
                message = await websocket.recv()
                data = json.loads(message)
                imu = data["imu"]
                process_imu_data(imu["accelX"], imu["accelY"], imu["accelZ"], imu["gyroZ"])
            except Exception as e:
                print(f"Error: {e}")
                break

async def run_websocket():
    await receive_data()

# Start WebSocket Thread
thread = threading.Thread(target=asyncio.run, args=(run_websocket(),))
thread.start()

# Start Animation
ani = animation.FuncAnimation(fig, update_plot, interval=20)
plt.show()
