import tkinter as tk
from tkinter import messagebox
from websocket import create_connection
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection
from matplotlib.patches import Polygon
import numpy as np
import asyncio
import websockets
import threading
import json
from datetime import datetime
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
dt = 0.1  #  update rate
ALPHA_GYRO = 0.995  # Trust more on gyro, but correct drift
ACCEL_ALPHA = 0.3  # Stronger smoothing for acceleration
VEL_DECAY = 0.96  # Reduce oscillations by velocity decay
STATIONARY_THRESHOLD = 0.03  # Threshold for zero-velocity update (ZUPT)

# State Variables
X, Y = 0.0, 0.0  # Global position
Vx, Vy = 0.0, 0.0  # Global velocity
yaw = 0.0  # Orientation
gyroZ_filtered = 0.0  # Filtered gyro
ax_m, ay_m = 0.0, 0.0  # Filtered acceleration

# Data storage for plotting
x_data, y_data = [0], [0]
# Global variables for scan data
scan_x_data, scan_y_data = [], []  # Store X and Y coordinates of scan data
# Global list to store polygons
scan_polygons = []  # Each element is a tuple: (polygon_x, polygon_y)

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

# Kalman Filter for yaw estimation
kf_yaw = KalmanFilter(dim_x=2, dim_z=1)  # State: [yaw, yaw_rate], Measurement: [yaw]
kf_yaw.F = np.array([[1, dt],  # State transition matrix
                     [0, 1]])
kf_yaw.H = np.array([[1, 0]])  # Measurement function
kf_yaw.P *= 10  # Initial covariance matrix
kf_yaw.R = np.array([[0.1]])  # Measurement noise covariance (sensor noise)
kf_yaw.Q = np.array([[0.01, 0],  # Process noise covariance (gyro noise)
                     [0, 0.01]])
kf_yaw.x = np.array([0, 0])  # Initial state: [yaw, yaw_rate]


# Add a global variable to track repeated acceleration values
last_accel = None  # Store the last acceleration value
repeat_count = 0  # Count how many times the same value is repeated
REPEAT_THRESHOLD = 5  # Number of iterations to tolerate repeated values

def process_imu_data(accelX, accelY, accelZ, gyroX, gyroY, gyroZ, imu_yaw):
    """Process IMU data to compute the car's position and estimate yaw."""
    global X, Y, Vx, Vy, yaw, gyroZ_filtered, ax_m, ay_m, quat, position, velocity
    global last_accel, repeat_count, kf_yaw   # Use the global variables for tracking

    try:
        # Convert gyroscope data from degrees per second to radians per second
        gyro = np.array([gyroX, gyroY, gyroZ]) * (np.pi / 180)  # [rad/s]
        accel = np.array([accelX, accelY, accelZ])  # [m/s^2]

        # Ignore updates if IMU data is too small
        if np.linalg.norm(gyro) < 1e-6 and np.linalg.norm(accel) < 1e-6:
            # print("IMU data too small, skipping update.")
            return

        # Check for repeated acceleration values
        if last_accel is not None and np.allclose(accel, last_accel, atol=1e-3):
            repeat_count += 1
            if repeat_count > REPEAT_THRESHOLD:
                # print("Repeated acceleration detected, skipping update.")
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

        # Extract global acceleration components
        ax_global, ay_global = acc_global[0], acc_global[1]

        # Zero-Velocity Update (ZUPT)
        if abs(ax_global) < STATIONARY_THRESHOLD and abs(ay_global) < STATIONARY_THRESHOLD:
            Vx, Vy = 0, 0  # Reset velocity to zero
        else:
            # Update velocity and position
            velocity += acc_global * dt

        # Apply velocity decay to reduce drift
        velocity *= VEL_DECAY

        # Update position only if velocity is above a threshold
        VELOCITY_THRESHOLD = 0.02
        if np.linalg.norm(velocity[:2]) > VELOCITY_THRESHOLD:
            position += velocity * dt

        # Kalman Filter for Yaw
        imu_yaw_radians = np.radians(imu_yaw)  # Convert imu_yaw to radians
        gyro_yaw_rate = gyro[2]  # Gyroscope yaw rate (rad/s)

        # Prediction Step
        kf_yaw.predict()

        # Update Step
        kf_yaw.update(np.array([imu_yaw_radians]))  # Use sensor yaw as measurement

        # Extract the filtered yaw and yaw rate
        yaw, yaw_rate = kf_yaw.x

        # Normalize yaw to the range [-pi, pi]
        yaw = (yaw + np.pi) % (2 * np.pi) - np.pi
        yaw = yaw + np.pi/2  # Adjust yaw to match the car's orientation
        # yaw = yaw*1.2/5  # Adjust yaw to match the car's orientation
        # Update the heading arrow
        dx = np.cos(yaw)  # X component of the heading
        dy = np.sin(yaw)  # Y component of the heading
        heading_arrow.set_UVC(dx, dy)  # Update arrow direction
        heading_arrow.set_offsets([[X, Y]])  # Update arrow position


        # Kalman Filter Prediction Step
        kf.predict()

        # Kalman Filter Update Step
        measurement = np.array([position[0], position[1]])  # Use position as measurement
        kf.update(measurement)

        # Extract filtered position and velocity
        X, Y, Vx, Vy = kf.x[0], kf.x[1], kf.x[2], kf.x[3]

        x_data.append(X)
        y_data.append(Y)



    except ValueError as e:
        print(f"Error in process_imu_data: {e}")
        log_message(f"Error in process_imu_data: {e}")

def plot_scanning_data(angles, distances):
    """Plot scanning data at the latest position."""
    global X, Y, yaw, scan_x_data, scan_y_data, scan_line, scan_polygons

    # Ensure angles and distances are valid sequences
    if not angles or not distances or len(angles) != len(distances):
        print("Warning: Invalid angles or distances. Skipping scanning plot.")
        return

     # Convert angles and distances to global coordinates
    scan_x_data = []
    scan_y_data = []

    # Convert angles and distances to global coordinates
    for angle, distance in zip(angles, distances):
        angle_rad = np.radians(angle) + yaw - np.pi/2  # Convert angle to radians and add current yaw
        x_scan = X + distance * np.cos(angle_rad)  # Calculate global X
        y_scan = Y + distance * np.sin(angle_rad)  # Calculate global Y
        scan_x_data.append(x_scan)
        scan_y_data.append(y_scan)

    # Create a polygon by joining the car's current position and the scan points
    polygon_x = [X] + scan_x_data + [X]  # Close the polygon by returning to the car's position
    polygon_y = [Y] + scan_y_data + [Y]
    scan_polygons.append((polygon_x, polygon_y))  # Store the polygon for later use

    # Draw only the latest polygon
    ax.fill(polygon_x, polygon_y, color='lightblue', alpha=0.5)

     # Update the heading arrow

    # Update the scan line plot
    # scan_line.set_data(scan_x_data, scan_y_data)

        # Adjust plot limits dynamically
    if len(scan_x_data) > 10:
        x_min, x_max = min(scan_x_data), max(scan_x_data)
        y_min, y_max = min(scan_y_data), max(scan_y_data)
        padding = 1  # Add padding for better view
        ax.set_xlim(x_min - padding, x_max + padding)
        ax.set_ylim(y_min - padding, y_max + padding)

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
                        process_imu_data(imu["aX"], -1*imu["aY"], imu["aZ"], imu["gX"], imu["gY"], imu["gZ"], -1*imu["Y"])
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
                            safe_run_coroutine(send_command("S"))  # Stop command
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


def restart():
    """Restart the application by resetting all variables and connections."""
    global ws_CMD, ws_DATA, X, Y, Vx, Vy, yaw, gyroZ_filtered, ax_m, ay_m, quat, position, velocity
    global x_data, y_data, kf, kf_yaw

    try:
        # Close WebSocket connections
        if ws_CMD:
            asyncio.run_coroutine_threadsafe(ws_CMD.close(), asyncio_loop)
            ws_CMD = None
        if ws_DATA:
            asyncio.run_coroutine_threadsafe(ws_DATA.close(), asyncio_loop)
            ws_DATA = None

        # Reset Kalman filters
        kf.x = np.array([0, 0, 0, 0])  # Reset state: [X, Y, Vx, Vy]
        kf.P *= 1000  # Reset covariance matrix
        kf_yaw.x = np.array([0, 0])  # Reset state: [yaw, yaw_rate]
        kf_yaw.P *= 10  # Reset covariance matrix

        # Reset state variables
        X, Y = 0.0, 0.0
        Vx, Vy = 0.0, 0.0
        yaw = 0.0
        gyroZ_filtered = 0.0
        ax_m, ay_m = 0.0, 0.0
        quat = np.array([1.0, 0.0, 0.0, 0.0])  # Reset quaternion
        position = np.zeros(3)  # Reset position [x, y, z]
        velocity = np.zeros(3)  # Reset velocity [vx, vy, vz]

        # Reset plot data
        clear_plot()

        # Log the restart
        log_message("Application restarted successfully.")
        print("Application restarted successfully.")

    except Exception as e:
        log_message(f"Error during restart: {e}")
        print(f"Error during restart: {e}")
        messagebox.showerror("Restart Error", f"Failed to restart the application: {e}")


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
        safe_show_warning("Warning", "Not connected to the car!")

def safe_show_error(title, message):
    """Show an error message in a thread-safe way."""
    root.after(0, lambda: messagebox.showerror(title, message))

def safe_show_warning(title, message):
    """Show a warning message in a thread-safe way."""
    root.after(0, lambda: messagebox.showwarning(title, message))

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
    """Log a message in the GUI with a timestamp."""
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    log_text.insert(tk.END, f"[{timestamp}] {message}\n")
    log_text.see(tk.END)

def on_key_press(event):
    """Handle key press events to control the car."""
    try:
        key = event.keysym
        if key == "Up":
            safe_run_coroutine(send_command("F"))  # Forward
        elif key == "Down":
            safe_run_coroutine(send_command("B"))  # Backward
        elif key == "Left":
            safe_run_coroutine(send_command("L"))  # Left
        elif key == "Right":
            safe_run_coroutine(send_command("R"))  # Right
        elif key == "space" or key == "c":
            safe_run_coroutine(send_command("C"))  # Scan
    except Exception as e:
        print(f"Error handling key press: {e}")
        log_message(f"Error handling key press: {e}")

def on_key_release(event):
    """Handle key release events to stop the car."""
    try:
        safe_run_coroutine(send_command("S"))  # Stop
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

def clear_plot():
    """Clear the plot and reset the data."""
    global x_data, y_data, scan_x_data, scan_y_data

    # Reset the data
    x_data, y_data = [0], [0]
    scan_x_data, scan_y_data = [], []

    # Clear the line on the plot
    line_path.set_data([], [])
    scan_line.set_data([], [])

    # Remove all filled areas (polygons)
    for collection in ax.collections[:]:
        if isinstance(collection, PolyCollection):  # Check if it's a polygon
            collection.remove()

    # Clear the starting and current location markers
    start_marker.set_data([], [])
    current_marker.set_data([], [])

    # Reset the heading arrow
    heading_arrow.set_UVC(0, 1)  # Reset the arrow direction
    heading_arrow.set_offsets([[x_data[0], y_data[0]]])  # Reset to the starting position

    # Reset the plot limits
    ax.relim()
    ax.autoscale_view()

    # Redraw the canvas
    canvas.draw()

    # Log the action
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
scan_line, = ax.plot([], [], 'r-', label="Scan Path")  # Red line for scan data
# Initialize markers for starting and current locations
start_marker, = ax.plot([], [], 'go', label="Start", markersize=10)  # Green marker for start
current_marker, = ax.plot([], [], 'ro', label="Current", markersize=10)  # Red marker for current
ax.legend()

# Initialize heading arrow
heading_arrow = ax.quiver(0, 0, 0, 1, angles='xy', scale_units='xy', scale=5, color='red', label="Heading")

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
        messagebox.showerror("Error", f"Failed to execute coroutine: {e}")
        
# Connection frame
connection_frame = tk.Frame(control_frame)
connection_frame.pack(pady=10)

connect_button = tk.Button(connection_frame, text="Connect", command=lambda: safe_run_coroutine(connect_to_car()))
connect_button.pack(side=tk.LEFT, padx=5)

disconnect_button = tk.Button(connection_frame, text="Disconnect", command= lambda: safe_run_coroutine(close_all_connections_sync()))
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

# Restart Button
restart_frame = tk.Frame(control_frame)
restart_frame.pack(pady=5)

restart_button = tk.Button(restart_frame, text="Restart", command=restart)
restart_button.pack(side=tk.LEFT, padx=5)

# Start the asyncio event loop in a separate thread
asyncio_thread = threading.Thread(target=start_asyncio_loop, daemon=True)
asyncio_thread.start()

# Wait for the asyncio loop to be ready
asyncio_loop_ready.wait()



# Start the WebSocket threads
start_websocket_threads()

# Start the GUI loop
root.mainloop()

