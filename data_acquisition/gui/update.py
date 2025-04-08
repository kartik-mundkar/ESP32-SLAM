import tkinter as tk
from tkinter import messagebox
from websocket import create_connection
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.pyplot as plt
import numpy as np
import asyncio
import websockets
import threading
import json
import time

# WebSocket connection details
CAR_IP = "192.168.137.120"  # Replace with your ESP32's IP address
CAR_CMD_PORT = 81           # WebSocket server port for commands
CAR_DATA_PORT = 82          # WebSocket server port for data
WS_URL_CMD = f"ws://{CAR_IP}:{CAR_CMD_PORT}"
WS_URL_DATA = f"ws://{CAR_IP}:{CAR_DATA_PORT}"

# WebSocket clients
ws_CMD = None
ws_DATA = None
last_command = None  # Store the last sent command

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
STATIONARY_THRESHOLD = 0.02  # Threshold for zero-velocity update (ZUPT)

# State Variables
X, Y = 0.0, 0.0  # Global position
Vx, Vy = 0.0, 0.0  # Global velocity
yaw = 0.0  # Orientation
gyroZ_filtered = 0.0  # Filtered gyro
ax_m, ay_m = 0.0, 0.0  # Filtered acceleration

# Data storage for plotting
x_data, y_data = [0], [0]

def process_imu_data(accelX, accelY, gyroZ):
    """Process IMU data to compute the car's position."""
    global X, Y, Vx, Vy, yaw, gyroZ_filtered, ax_m, ay_m

    # Filter gyroscope data
    gyroZ_filtered = ALPHA_GYRO * (gyroZ_filtered + gyroZ * dt) + (1 - ALPHA_GYRO) * gyroZ
    yaw += gyroZ_filtered * dt  # Integrate yaw rate to get orientation

    # Filter accelerometer data
    ax_m = (1 - ACCEL_ALPHA) * ax_m + ACCEL_ALPHA * (accelX * g)
    ay_m = (1 - ACCEL_ALPHA) * ay_m + ACCEL_ALPHA * (accelY * g)

    # Transform acceleration to global frame
    ax_global = ax_m * np.cos(yaw) - ay_m * np.sin(yaw)
    ay_global = ax_m * np.sin(yaw) + ay_m * np.cos(yaw)

    # Zero-velocity update (ZUPT)
    if abs(ax_global) < STATIONARY_THRESHOLD and abs(ay_global) < STATIONARY_THRESHOLD:
        Vx, Vy = 0, 0
    else:
        # Update velocity and position
        Vx = Vx * VEL_DECAY + ax_global * dt
        Vy = Vy * VEL_DECAY + ay_global * dt
        X += Vx * dt
        Y += Vy * dt

    # Store position for plotting
    x_data.append(X)
    y_data.append(Y)

def update_plot():
    """Update the Matplotlib plot in the GUI."""
    line_path.set_data(x_data, y_data)
    if len(x_data) > 10:
        x_min, x_max = min(x_data), max(x_data)
        y_min, y_max = min(y_data), max(y_data)
        padding = 1  # Add padding for better view
        ax.set_xlim(x_min - padding, x_max + padding)
        ax.set_ylim(y_min - padding, y_max + padding)
    canvas.draw()

async def receive_data():
    """WebSocket Client to receive IMU data."""
    global ws_DATA
    while True:  # Keep trying to connect
        try:
            ws_DATA = await websockets.connect(WS_URL_DATA)
            print("Connected to ESP32 WebSocket for data!")
            log_message("Connected to ESP32 WebSocket for data!")
            while True:
                try:
                    message = await ws_DATA.recv()
                    data = json.loads(message)
                    if data["mode"] == "moving":
                        # Process IMU data
                        imu = data["car"]["imu"]
                        print(f"IMU data: {imu}")
                        process_imu_data(imu["aX"], imu["aY"], imu["gZ"])
                        update_plot()
                    elif data["mode"] == "scanning":
                        # Process ultra data for scanning mode
                        ultra = data["scan"]
                        angles = ultra["angle"]
                        distances = ultra["distance"]
                        # Here you can add code to visualize the scanning data if needed
                        # For example, you can plot the distances at the corresponding angles
                        # This is just a placeholder for demonstration
                        print(f"Scanning data: Angles: {angles}, Distances: {distances}")
                    else:
                        pass  # Ignore other modes
                    
                except (websockets.ConnectionClosedError, websockets.ConnectionClosedOK, OSError) as e:
                    print(f"Data WebSocket disconnected: {e}. Reconnecting...")
                    log_message(f"Data WebSocket disconnected: {e}. Reconnecting...")
                    break  # Exit the inner loop to reconnect
        except Exception as e:
            print(f"Failed to connect to data WebSocket: {e}. Retrying...")
            log_message(f"Failed to connect to data WebSocket: {e}. Retrying...")
            await asyncio.sleep(2)  # Wait before retrying

def start_websocket_thread():
    """Start the WebSocket thread for receiving data."""
    thread = threading.Thread(target=asyncio.run, args=(receive_data(),))
    thread.daemon = True  # Daemonize thread to exit when main program exits
    thread.start()

def connect_to_car():
    """Connect to the car's WebSocket server for commands."""
    global ws_CMD
    while True:
        try:
            ws_CMD = create_connection(WS_URL_CMD)
            messagebox.showinfo("Connection", "Connected to the car!")
            log_message("Connected to the car!")
            break
        except Exception as e:
            print(f"Command WebSocket connection failed: {e}. Retrying...")
            log_message(f"Command WebSocket connection failed: {e}. Retrying...")
            time.sleep(2)  # Retry after a short delay

def send_command(command):
    """Send a command to the car."""
    global ws_CMD, last_command
    if command == last_command:
        return  # Skip sending the command if it's the same as the last one
    if ws_CMD:
        try:
            ws_CMD.send(command)
            log_message(f"Sent command: {command}")
            last_command = command  # Update the last command
        except (websockets.ConnectionClosedError, websockets.ConnectionClosedOK, OSError) as e:
            # print(f"Command WebSocket disconnected: {e}. Reconnecting...")
            log_message("Command WebSocket disconnected. Reconnecting...")
            connect_to_car()
        except Exception as e:
            messagebox.showerror("Error", f"Failed to send command: {e}")
    else:
        messagebox.showwarning("Warning", "Not connected to the car!")


def send_speed():
    """Send the speed to the car."""
    global ws_CMD
    if ws_CMD:
        try:
            speed = speed_entry.get()
            if not speed.isdigit():
                messagebox.showerror("Invalid Input", "Speed must be a positive integer.")
                return
            ws_CMD.send(f"SPEED:{speed}")
            log_message(f"Sent speed: {speed}")
        except Exception as e:
            messagebox.showerror("Error", f"Failed to send speed: {e}")
    else:
        messagebox.showwarning("Warning", "Not connected to the car!")

def send_servo_angle():
    """Send the servo angle to the car."""
    global ws_CMD
    if ws_CMD:
        try:
            angle = angle_entry.get()
            if not angle.isdigit() or not (0 <= int(angle) <= 180):
                messagebox.showerror("Invalid Input", "Angle must be an integer between 0 and 180.")
                return
            ws_CMD.send(f"ANGLE:{angle}")
            log_message(f"Sent servo angle: {angle}")
        except Exception as e:
            messagebox.showerror("Error", f"Failed to send servo angle: {e}")
    else:
        messagebox.showwarning("Warning", "Not connected to the car!")

def send_calibration_command():
    """Send a calibration command to the car."""
    global ws_CMD
    if ws_CMD:
        try:
            ws_CMD.send("CALIBRATE")
            log_message("Sent calibration command to the car.")
        except Exception as e:
            messagebox.showerror("Error", f"Failed to send calibration command: {e}")
    else:
        messagebox.showwarning("Warning", "Not connected to the car!")



def log_message(message):
    """Log a message in the GUI."""
    log_text.insert(tk.END, message + "\n")
    log_text.see(tk.END)

def on_key_press(event):
    """Handle key press events to control the car."""
    key = event.keysym
    if key == "Up":
        send_command("F")  # Forward
    elif key == "Down":
        send_command("B")  # Backward
    elif key == "Left":
        send_command("L")  # Left
    elif key == "Right":
        send_command("R")  # Right
    elif key == "space" or key == "c":
        send_command("C")  # Scan
    

def on_key_release(event):
    """Handle key release events to stop the car."""
    send_command("S")  # Stop


def close_all_connections():
    """Close all WebSocket connections when the UI is closed."""
    global ws_CMD, ws_DATA
    if ws_CMD:
        try:
            ws_CMD.close()
        except Exception as e:
            print(f"Error closing command WebSocket: {e}")
    if ws_DATA:
        try:
            asyncio.run(ws_DATA.close())
        except Exception as e:
            print(f"Error closing data WebSocket: {e}")
    root.destroy()


# GUI setup
root = tk.Tk()
root.title("ESP32 Car Controller with Real-Time Plot")

# Bind keyboard events
root.bind("<KeyPress>", on_key_press)
root.bind("<KeyRelease>", on_key_release)


# Main layout frames
main_frame = tk.Frame(root)
main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10, side=tk.TOP)
root.geometry("1200x600")
root.protocol("WM_DELETE_WINDOW", close_all_connections)  # Handle window close event

# Left frame for the plot
plot_frame = tk.Frame(main_frame)
plot_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=10, pady=10)

# Right frame for controls
control_frame = tk.Frame(main_frame)
control_frame.pack(side=tk.RIGHT, fill=tk.Y, padx=10, pady=10)

# Matplotlib plot
fig, ax = plt.subplots(figsize=(5, 5))
ax.set_title("Real-Time Car Path")
ax.set_xlabel("X Position (m)")
ax.set_ylabel("Y Position (m)")
ax.grid(True)
line_path, = ax.plot([], [], 'bo-', label="Car Path")
ax.legend()

canvas = FigureCanvasTkAgg(fig, master=plot_frame)
canvas_widget = canvas.get_tk_widget()
canvas_widget.pack(fill=tk.BOTH, expand=True)

# Connection frame
connection_frame = tk.Frame(control_frame)
connection_frame.pack(pady=10)

connect_button = tk.Button(connection_frame, text="Connect", command=connect_to_car)
connect_button.pack(side=tk.LEFT, padx=5)

disconnect_button = tk.Button(connection_frame, text="Disconnect", command=lambda: send_command("DISCONNECT"))
disconnect_button.pack(side=tk.LEFT, padx=5)

# Command and Input Frames Side by Side
command_input_frame = tk.Frame(control_frame)
command_input_frame.pack(pady=10, fill=tk.X)

# Subframe for Command Buttons
command_frame = tk.Frame(command_input_frame)
command_frame.pack(side=tk.LEFT, padx=10)

forward_button = tk.Button(command_frame, text="Forward", command=lambda: send_command("F"))
forward_button.grid(row=0, column=1, padx=5, pady=5)

backward_button = tk.Button(command_frame, text="Backward", command=lambda: send_command("B"))
backward_button.grid(row=2, column=1, padx=5, pady=5)

left_button = tk.Button(command_frame, text="Left", command=lambda: send_command("L"))
left_button.grid(row=1, column=0, padx=5, pady=5)

right_button = tk.Button(command_frame, text="Right", command=lambda: send_command("R"))
right_button.grid(row=1, column=2, padx=5, pady=5)

stop_button = tk.Button(command_frame, text="Stop", command=lambda: send_command("S"))
stop_button.grid(row=1, column=1, padx=5, pady=5)

scan_button = tk.Button(command_frame, text="Scan", command=lambda: send_command("C"))
scan_button.grid(row=3, column=1, padx=5, pady=5)

# Subframe for Input Frames and Calibration
input_frame = tk.Frame(command_input_frame)
input_frame.pack(side=tk.LEFT, padx=10)

# Speed Input Frame
speed_frame = tk.Frame(input_frame)
speed_frame.pack(pady=5)

speed_label = tk.Label(speed_frame, text="Speed:")
speed_label.pack(side=tk.LEFT, padx=5)

speed_entry = tk.Entry(speed_frame, width=10)
speed_entry.pack(side=tk.LEFT, padx=5)

speed_button = tk.Button(speed_frame, text="Set Speed", command=send_speed)
speed_button.pack(side=tk.LEFT, padx=5)

# Servo Angle Input Frame
angle_frame = tk.Frame(input_frame)
angle_frame.pack(pady=5)

angle_label = tk.Label(angle_frame, text="Servo Angle:")
angle_label.pack(side=tk.LEFT, padx=5)

angle_entry = tk.Entry(angle_frame, width=10)
angle_entry.pack(side=tk.LEFT, padx=5)

angle_button = tk.Button(angle_frame, text="Set Angle", command=send_servo_angle)
angle_button.pack(side=tk.LEFT, padx=5)

# Calibration Button
calibration_frame = tk.Frame(input_frame)
calibration_frame.pack(pady=5)

calibrate_button = tk.Button(calibration_frame, text="Calibrate IMU", command=send_calibration_command)
calibrate_button.pack(side=tk.LEFT, padx=5)

# Log frame
log_frame = tk.Frame(control_frame)
log_frame.pack(pady=10)

log_label = tk.Label(log_frame, text="Log:")
log_label.pack(anchor=tk.W)

log_text = tk.Text(log_frame, height=10, width=50, state=tk.NORMAL)
log_text.pack()

# Start the WebSocket thread for data
start_websocket_thread()

# Automatically connect to the car at startup
threading.Thread(target=connect_to_car, daemon=True).start()

# Start the GUI loop
root.mainloop()