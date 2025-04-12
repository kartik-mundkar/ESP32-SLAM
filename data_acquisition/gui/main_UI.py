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
from ahrs.filters import Madgwick
from scipy.spatial.transform import Rotation as R
from filterpy.kalman import KalmanFilter

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

# Initialize Madgwick filter and state variables
madgwick = Madgwick()
quat = np.array([1.0, 0.0, 0.0, 0.0])  # Initial quaternion
position = np.zeros(3)  # Global position [x, y, z]
velocity = np.zeros(3)  # Global velocity [vx, vy, vz]

# Initialize Kalman Filter for position and velocity estimation
kf = KalmanFilter(dim_x=4, dim_z=2)  # State: [X, Y, Vx, Vy], Measurement: [X, Y]
kf.F = np.array([[1, 0, dt, 0],  # State transition matrix
                 [0, 1, 0, dt],
                 [0, 0, 1, 0],
                 [0, 0, 0, 1]])
kf.H = np.array([[1, 0, 0, 0],  # Measurement function
                 [0, 1, 0, 0]])
kf.P *= 1000  # Initial covariance matrix
kf.R = np.array([[0.1, 0],  # Measurement noise covariance
                 [0, 0.1]])
kf.Q = np.array([[0.01, 0, 0, 0],  # Process noise covariance
                 [0, 0.01, 0, 0],
                 [0, 0, 0.1, 0],
                 [0, 0, 0, 0.1]])
kf.x = np.array([0, 0, 0, 0])  # Initial state: [X, Y, Vx, Vy]

# Add a global variable to track repeated acceleration values
last_accel = None  # Store the last acceleration value
repeat_count = 0  # Count how many times the same value is repeated
REPEAT_THRESHOLD = 10  # Number of iterations to tolerate repeated values

def process_imu_data(accelX, accelY, accelZ, gyroX, gyroY, gyroZ, imu_yaw):
    """Process IMU data to compute the car's position and estimate yaw."""
    global X, Y, Vx, Vy, yaw, gyroZ_filtered, ax_m, ay_m, quat, position, velocity
    global last_accel, repeat_count  # Use the global variables for tracking

    try:
        # Convert gyroscope data from degrees per second to radians per second
        gyro = np.array([gyroX, gyroY, gyroZ]) * (np.pi / 180)  # [rad/s]
        accel = np.array([accelX, accelY, accelZ])  # [m/s^2]

        # Ignore updates if IMU data is too small
        if np.linalg.norm(gyro) < 1e-6 and np.linalg.norm(accel) < 1e-6:
            print("IMU data too small, skipping update.")
            return

        # Check for repeated acceleration values
        if last_accel is not None and np.allclose(accel, last_accel, atol=1e-3):
            repeat_count += 1
            if repeat_count > REPEAT_THRESHOLD:
                print("Repeated acceleration detected, skipping update.")
                return
        else:
            repeat_count = 0  # Reset the counter if the value changes

        # Update the last acceleration value
        last_accel = accel

        # Update quaternion using Madgwick filter
        quat = madgwick.updateIMU(quat, gyr=gyro, acc=accel)

        # Normalize the quaternion to avoid zero norm issues
        quat_norm = np.linalg.norm(quat)
        if quat_norm == 0:
            print("Warning: Quaternion norm is zero. Resetting quaternion.")
            quat[:] = np.array([1.0, 0.0, 0.0, 0.0])  # Reset to identity quaternion
        else:
            quat /= quat_norm

        # Convert quaternion to rotation matrix
        rotation = R.from_quat(quat)
        acc_global = rotation.apply(accel)  # Transform acceleration to global frame

        # Remove gravity from the Z-axis
        acc_global[2] -= g  # Subtract gravity (9.81 m/s^2)

        # Zero-Velocity Update (ZUPT)
        if np.linalg.norm(acc_global[:2]) < STATIONARY_THRESHOLD:
            velocity[:] = 0  # Reset velocity to zero
        else:
            # Update velocity and position
            velocity += acc_global * dt
            position += velocity * dt

        # Apply velocity decay to reduce drift
        velocity *= VEL_DECAY

        # Estimate yaw using gyroscope data
        yaw += gyro[2] * dt  # Integrate gyroscope Z-axis data to estimate yaw

        # Correct yaw using the sensor-provided yaw (imu_yaw)
        imu_yaw_radians = np.radians(imu_yaw)  # Convert imu_yaw to radians
        yaw = ALPHA_GYRO * yaw + (1 - ALPHA_GYRO) * imu_yaw_radians  # Complementary filter

        # Normalize yaw to the range [-pi, pi]
        yaw = (yaw + np.pi) % (2 * np.pi) - np.pi

        # Kalman Filter Prediction Step
        kf.predict()

        # Kalman Filter Update Step
        measurement = np.array([position[0], position[1]])  # Use position as measurement
        kf.update(measurement)

        # Extract filtered position and velocity
        X, Y, Vx, Vy = kf.x[0], kf.x[1], kf.x[2], kf.x[3]

        # Store position for plotting
        x_data.append(X)
        y_data.append(Y)

    except ValueError as e:
        print(f"Error in process_imu_data: {e}")
        log_message(f"Error in process_imu_data: {e}")

def plot_scanning_data(angles, distances):
    """Plot scanning data at the latest position."""
    global X, Y, yaw

    # Ensure angles and distances are valid sequences
    if not angles or not distances or len(angles) != len(distances):
        print("Warning: Invalid angles or distances. Skipping scanning plot.")
        return

    # Convert angles and distances to global coordinates
    for angle, distance in zip(angles, distances):
        angle_rad = np.radians(angle) + yaw  # Convert angle to radians and add current yaw
        x_scan = X + distance * np.cos(angle_rad)  # Calculate global X
        y_scan = Y + distance * np.sin(angle_rad)  # Calculate global Y
        ax.plot(x_scan, y_scan, 'ro', markersize=2)  # Plot as red dots for scanning data

    canvas.draw()  # Update the plot

def update_plot():
    """Update the Matplotlib plot in the GUI."""
    # Ensure x_data and y_data are valid sequences
    if len(x_data) == 0 or len(y_data) == 0:
        print("Warning: x_data or y_data is empty. Skipping plot update.")
        return

    # Update the car's path
    line_path.set_data(x_data, y_data)

    # Update the starting location marker
    if len(x_data) > 0 and len(y_data) > 0:
        start_marker.set_data([x_data[0]], [y_data[0]])  # Starting location

    # Update the current location marker
    if len(x_data) > 0 and len(y_data) > 0:
        current_marker.set_data([x_data[-1]], [y_data[-1]])  # Current location

     # Update the heading arrow
    if len(x_data) > 0 and len(y_data) > 0:
        # Calculate the arrow direction based on yaw
        dx = np.cos(yaw)  # X component of the heading
        dy = np.sin(yaw)  # Y component of the heading
        heading_arrow.set_UVC(dx, dy)  # Update arrow direction
        heading_arrow.set_offsets([[x_data[-1], y_data[-1]]])  # Update arrow position

    # Adjust plot limits dynamically
    if len(x_data) > 10:
        x_min, x_max = min(x_data), max(x_data)
        y_min, y_max = min(y_data), max(y_data)
        padding = 1  # Add padding for better view
        ax.set_xlim(x_min - padding, x_max + padding)
        ax.set_ylim(y_min - padding, y_max + padding)

    # Redraw the canvas
    canvas.draw()

async def receive_data():
    """WebSocket Client to receive IMU data."""
    global ws_DATA
    while True:  # Keep trying to connect
        try:
            ws_DATA = await websockets.connect(WS_URL_DATA)
            update_data_status(True)  # Update UI status
            print("Connected to ESP32 WebSocket for data!")
            log_message("Connected to ESP32 WebSocket for data!")
            while True:
                try:
                    message = await ws_DATA.recv()
                    data = json.loads(message)
                    if data["mode"] == "moving":
                        # Process IMU data
                        imu = data["car"]["imu"]
                        ultra = data["car"]["ultra"]
                        print(f"IMU: {imu}, Ultra: {ultra}")
                        process_imu_data(imu["aX"], -1*imu["aY"], imu["aZ"], imu["gX"], imu["gY"], imu["gZ"], imu["Y"])
                        update_plot()
                    elif data["mode"] == "scanning":
                        # Process ultra data for scanning mode
                        ultra = data["scan"]
                        angles = ultra["angle"]
                        distances = ultra["distance"]
                        plot_scanning_data(angles, distances)
                        print(f"Scanning data: Angles: {angles}, Distances: {distances}")
                    else:
                        pass  # Ignore other modes
                except (websockets.ConnectionClosedError, websockets.ConnectionClosedOK, OSError) as e:
                    update_data_status(False)  # Update UI status
                    print(f"Data WebSocket disconnected: {e}. Reconnecting...")
                    log_message(f"Data WebSocket disconnected: {e}. Reconnecting...")
                    if ws_CMD:
                        try:
                            # Schedule the coroutine to send the stop command
                            asyncio.run_coroutine_threadsafe(ws_CMD.send("S"), asyncio_loop)
                        except Exception as e:
                            print(f"Error sending stop command: {e}")
                    await asyncio.sleep(2)  # Wait before retrying
                    break  # Exit the inner loop to reconnect
        except Exception as e:
            update_data_status(False)  # Update UI status
            print("here is the error")

            print(f"Failed to connect to data WebSocket: {e}. Retrying...")
            log_message(f"Failed to connect to data WebSocket: {e}. Retrying...")
            await asyncio.sleep(2)  # Wait before retrying

async def connect_to_car():
    """Connect to the car's WebSocket server for commands and handle reconnection."""
    global ws_CMD
    while True:  # Keep trying to connect
        try:
            ws_CMD = await websockets.connect(WS_URL_CMD)
            update_cmd_status(True)  # Update UI status
            print("Connected to ESP32 WebSocket for commands!")
            log_message("Connected to ESP32 WebSocket for commands!")

            while True:
                try:
                    await asyncio.sleep(1)  # Keep the connection alive
                    if ws_CMD is None:
                        break
                except (websockets.ConnectionClosedError, websockets.ConnectionClosedOK, OSError) as e:
                    update_cmd_status(False)  # Update UI status
                    print(f"Command WebSocket disconnected: {e}. Reconnecting...")
                    log_message(f"Command WebSocket disconnected: {e}. Reconnecting...")
                    break  # Exit the inner loop to reconnect
        except Exception as e:
            update_cmd_status(False)  # Update UI status
            print(f"Failed to connect to command WebSocket: {e}. Retrying...")
            log_message(f"Failed to connect to command WebSocket: {e}. Retrying...")
            await asyncio.sleep(2)  # Wait before retrying


# Event to signal that the asyncio loop is ready
asyncio_loop_ready = threading.Event()

def start_asyncio_loop():
    """Start the asyncio event loop in a separate thread."""
    global asyncio_loop
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    asyncio_loop = loop  # Store the event loop
    asyncio_loop_ready.set()  # Signal that the loop is ready
    loop.run_forever()

def start_websocket_threads():
    """Start threads for both WebSocket connections."""
    # Wait for the asyncio loop to be ready
    asyncio_loop_ready.wait()

    # Start the command WebSocket connection thread
    asyncio.run_coroutine_threadsafe(connect_to_car(), asyncio_loop)
    # Start the data WebSocket connection thread
    asyncio.run_coroutine_threadsafe(receive_data(), asyncio_loop)

async def send_command(command):
    """Send a command to the car."""
    global ws_CMD, last_command
    if command == last_command:
        return  # Avoid sending duplicate commands
    last_command = command
    if ws_CMD:
        try:
            await ws_CMD.send(command)  # Use await for async WebSocket
            update_current_command(command)  # Update the UI label
            log_message(f"Command sent: {command}")
        except Exception as e:
            log_message(f"Error sending command: {e}")
    else:
        messagebox.showwarning("Warning", "Not connected to the car!")

async def send_speed():
    """Send the speed to the car."""
    global ws_CMD
    if ws_CMD:
        try:
            speed = speed_entry.get()
            if not speed.isdigit():
                messagebox.showerror("Invalid Input", "Speed must be a positive integer.")
                return
            await ws_CMD.send(f"SPEED:{speed}")
            log_message(f"Sent speed: {speed}")
        except Exception as e:
            messagebox.showerror("Error", f"Failed to send speed: {e}")
    else:
        messagebox.showwarning("Warning", "Not connected to the car!")

async def send_servo_angle():
    """Send the servo angle to the car."""
    global ws_CMD
    if ws_CMD:
        try:
            angle = angle_entry.get()
            if not angle.isdigit() or not (0 <= int(angle) <= 180):
                messagebox.showerror("Invalid Input", "Angle must be an integer between 0 and 180.")
                return
            await ws_CMD.send(f"ANGLE:{angle}")
            log_message(f"Sent servo angle: {angle}")
        except Exception as e:
            messagebox.showerror("Error", f"Failed to send servo angle: {e}")
    else:
        messagebox.showwarning("Warning", "Not connected to the car!")

async def send_calibration_command():
    """Send a calibration command to the car."""
    global ws_CMD
    if ws_CMD:
        try:
            await ws_CMD.send("CALIBRATE")
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
    try:
        key = event.keysym
        if key == "Up":
            asyncio.run_coroutine_threadsafe(send_command("F"),asyncio_loop)  # Forward
        elif key == "Down":
            asyncio.run_coroutine_threadsafe(send_command("B"),asyncio_loop)  # Backward
        elif key == "Left":
            asyncio.run_coroutine_threadsafe(send_command("L"),asyncio_loop)  # Left
        elif key == "Right":
            asyncio.run_coroutine_threadsafe(send_command("R"),asyncio_loop)  # Right
        elif key == "space" or key == "c":
            asyncio.run_coroutine_threadsafe(send_command("C"),asyncio_loop)  # Scan
    except Exception as e:
        print(f"Error handling key press: {e}")
        log_message(f"Error handling key press: {e}")

def on_key_release(event):
    """Handle key release events to stop the car."""
    try:
        asyncio.run_coroutine_threadsafe(send_command("S"),asyncio_loop)  # Stop
    except Exception as e:
        print(f"Error handling key release: {e}")
        log_message(f"Error handling key release: {e}")

async def close_all_connections():
    """Close all WebSocket connections when the UI is closed."""
    global ws_CMD, ws_DATA
    if ws_CMD:
        try:
            await ws_CMD.close()
            print("Command WebSocket closed.")
        except Exception as e:
            print(f"Error closing command WebSocket: {e}")
    if ws_DATA:
        try:
            await ws_DATA.close()
            print("Data WebSocket closed.")
        except Exception as e:
            print(f"Error closing data WebSocket: {e}")
    root.destroy()

def save_plot(filename="car_path_plot.png"):
    """Save the current plot to a file with increased size."""
    try:
        # Temporarily increase the figure size for saving
        original_size = fig.get_size_inches()  # Store the original size
        fig.set_size_inches(10, 8)  # Set new size (width=10 inches, height=8 inches)
        
        fig.savefig(filename, dpi=300)  # Save the plot with high resolution
        log_message(f"Plot saved as {filename}")
        messagebox.showinfo("Save Plot", f"Plot successfully saved as {filename}")
        
        # Restore the original size after saving
        fig.set_size_inches(original_size)
    except Exception as e:
        log_message(f"Error saving plot: {e}")
        messagebox.showerror("Save Plot", f"Failed to save plot: {e}")

def smooth_data(data, window_size=5):
    """Apply a moving average filter to smooth the data."""
    smoothed = []
    for i in range(len(data)):
        start = max(0, i - window_size + 1)
        smoothed.append(sum(data[start:i + 1]) / (i - start + 1))
    return smoothed

def average_scan(angles, distances, num_scans=3):
    """Average multiple scans for each angle."""
    averaged_distances = [0] * len(distances)
    for _ in range(num_scans):
        for i, distance in enumerate(distances):
            averaged_distances[i] += distance
    return [d / num_scans for d in averaged_distances]

def remove_outliers(data, threshold=0.1):
    """Remove outliers from the data."""
    mean = sum(data) / len(data)
    return [d if abs(d - mean) <= threshold * mean else mean for d in data]

def process_scanning_data(angles, distances, num_scans=3, window_size=5):
    """Process scanning data to improve accuracy."""
    # Average multiple scans
    # distances = average_scan(angles, distances, num_scans=num_scans)
    # Smooth the data
    distances = smooth_data(distances, window_size=window_size)
    # Remove outliers
    distances = remove_outliers(distances)
    return distances

def clear_plot():
    """Clear the plot and reset the data."""
    global x_data, y_data
    x_data, y_data = [0], [0]  # Reset the data
    line_path.set_data([], [])  # Clear the line on the plot
    ax.relim()  # Reset the plot limits
    ax.autoscale_view()  # Autoscale the view
    canvas.draw()  # Redraw the canvas
    log_message("Plot cleared.")

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

def close_all_connections_sync():
    """Synchronous wrapper to close all WebSocket connections."""
    try:
        asyncio.run_coroutine_threadsafe(close_all_connections(), asyncio_loop)  # Use the dedicated event loop
    except Exception as e:
        print(f"Error closing connections: {e}")
        log_message(f"Error closing connections: {e}")

# Bind the synchronous wrapper to the window close event
root.protocol("WM_DELETE_WINDOW", close_all_connections_sync)

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
# Initialize markers for starting and current locations
start_marker, = ax.plot([], [], 'go', label="Start", markersize=10)  # Green marker for start
current_marker, = ax.plot([], [], 'ro', label="Current", markersize=10)  # Red marker for current
ax.legend()

# Initialize heading arrow
heading_arrow = ax.quiver(0, 0, 0, 0, angles='xy', scale_units='xy', scale=3, color='red', label="Heading")

canvas = FigureCanvasTkAgg(fig, master=plot_frame)
canvas_widget = canvas.get_tk_widget()
canvas_widget.pack(fill=tk.BOTH, expand=True)



def safe_run_coroutine(coro):
    """Safely run a coroutine in the asyncio event loop."""
    try:
        asyncio.run_coroutine_threadsafe(coro, asyncio_loop)
    except Exception as e:
        print(f"Error running coroutine: {e}")
        log_message(f"Error running coroutine: {e}")
        
# Connection frame
connection_frame = tk.Frame(control_frame)
connection_frame.pack(pady=10)

connect_button = tk.Button(connection_frame, text="Connect", command=lambda: asyncio.run_coroutine_threadsafe(connect_to_car(), asyncio_loop))
connect_button.pack(side=tk.LEFT, padx=5)

disconnect_button = tk.Button(connection_frame, text="Disconnect", command=lambda: lambda: asyncio.run_coroutine_threadsafe(send_command("DISCONNECT"), asyncio_loop))
disconnect_button.pack(side=tk.LEFT, padx=5)

# Add connection status labels
cmd_status_label = tk.Label(connection_frame, text="Command WS: Disconnected", fg="red")
cmd_status_label.pack(side=tk.LEFT, padx=5)

data_status_label = tk.Label(connection_frame, text="Data WS: Disconnected", fg="red")
data_status_label.pack(side=tk.LEFT, padx=5)

def update_cmd_status(connected):
    """Update the command WebSocket connection status."""
    if connected:
        cmd_status_label.config(text="Command WS: Connected", fg="green")
    else:
        cmd_status_label.config(text="Command WS: Disconnected", fg="red")

def update_data_status(connected):
    """Update the data WebSocket connection status."""
    if connected:
        data_status_label.config(text="Data WS: Connected", fg="green")
    else:
        data_status_label.config(text="Data WS: Disconnected", fg="red")


# Command and Input Frames Side by Side
command_input_frame = tk.Frame(control_frame)
command_input_frame.pack(pady=10, fill=tk.X)

# Subframe for Command Buttons
command_frame = tk.Frame(command_input_frame)
command_frame.pack(side=tk.LEFT, padx=10)

forward_button = tk.Button(command_frame, text="Forward", command=lambda: safe_run_coroutine(send_command("F")))
forward_button.grid(row=0, column=1, padx=5, pady=5)

backward_button = tk.Button(command_frame, text="Backward", command=lambda: safe_run_coroutine(send_command("B")))
backward_button.grid(row=2, column=1, padx=5, pady=5)

left_button = tk.Button(command_frame, text="Left", command=lambda: safe_run_coroutine(send_command("L")))
left_button.grid(row=1, column=0, padx=5, pady=5)

right_button = tk.Button(command_frame, text="Right", command=lambda: safe_run_coroutine(send_command("R")))
right_button.grid(row=1, column=2, padx=5, pady=5)

stop_button = tk.Button(command_frame, text="Stop", command=lambda: safe_run_coroutine(send_command("S")))
stop_button.grid(row=1, column=1, padx=5, pady=5)

scan_button = tk.Button(command_frame, text="Scan", command=lambda: safe_run_coroutine(send_command("C")))
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

speed_button = tk.Button(speed_frame, text="Set Speed", command=lambda: safe_run_coroutine(send_speed()))
speed_button.pack(side=tk.LEFT, padx=5)

# Servo Angle Input Frame
angle_frame = tk.Frame(input_frame)
angle_frame.pack(pady=5)

angle_label = tk.Label(angle_frame, text="Servo Angle:")
angle_label.pack(side=tk.LEFT, padx=5)

angle_entry = tk.Entry(angle_frame, width=10)
angle_entry.pack(side=tk.LEFT, padx=5)

angle_button = tk.Button(angle_frame, text="Set Angle", command=lambda: safe_run_coroutine(send_servo_angle()))
angle_button.pack(side=tk.LEFT, padx=5)

# Calibration Button
calibration_frame = tk.Frame(input_frame)
calibration_frame.pack(pady=5)

calibrate_button = tk.Button(calibration_frame, text="Calibrate IMU", command=lambda: safe_run_coroutine(send_calibration_command()))
calibrate_button.pack(side=tk.LEFT, padx=5)

# Save Plot Button
save_plot_frame = tk.Frame(control_frame)
save_plot_frame.pack(pady=5)

save_plot_button = tk.Button(save_plot_frame, text="Save Plot", command=lambda: save_plot("car_path_plot.png"))
save_plot_button.pack(side=tk.LEFT, padx=5)

# Add a label to display the current command
current_command_label = tk.Label(control_frame, text="Current Command: None", font=("Arial", 12), fg="blue")
current_command_label.pack(pady=5)

def update_current_command(command):
    """Update the current command label."""
    current_command_label.config(text=f"Current Command: {command}")

# Log frame
log_frame = tk.Frame(control_frame)
log_frame.pack(pady=10)

log_label = tk.Label(log_frame, text="Log:")
log_label.pack(anchor=tk.W)

log_text = tk.Text(log_frame, height=10, width=50, state=tk.NORMAL)
log_text.pack()


# Start the asyncio event loop in a separate thread
asyncio_thread = threading.Thread(target=start_asyncio_loop, daemon=True)
asyncio_thread.start()

# Wait for the asyncio loop to be ready
asyncio_loop_ready.wait()



# Start the WebSocket threads
start_websocket_threads()

# Start the GUI loop
root.mainloop()

