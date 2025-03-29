import asyncio
import websockets
import json

# Replace with your ESP32's IP Address
ESP32_WS_URL = "ws://192.168.137.210:81"

async def receive_data():
    async with websockets.connect(ESP32_WS_URL) as websocket:
        print("Connected to ESP32 WebSocket!")

        while True:
            try:
                # Receive WebSocket message
                message = await websocket.recv()
                data = json.loads(message)  # Convert JSON string to dictionary

                # Distinguish between Moving (IMU) and Scanning (Ultrasonic)
                imu = data["imu"]
                with open("Accel.csv",'+a') as file:
                    a = [imu['accelX'],imu['accelY'],imu['accelZ']]
                    string = ','.join(list(map(str,a)))
                    file.write(string)
                    file.write("\n")

                with open("Gyro.csv",'+a') as file:
                    a = [imu['gyroX'],imu['gyroY'],imu['gyroZ']]
                    string = ','.join(list(map(str,a)))
                    file.write(string)
                    file.write("\n")
                    
                # print(f"Accel X: {imu['accelX']} g, Accel Y: {imu['accelY']} g, Accel Z: {imu['accelZ']} g")
                # print(f"Gyro X: {imu['gyroX']} °/s, Gyro Y: {imu['gyroY']} °/s, Gyro Z: {imu['gyroZ']} °/s")
                # print()

            except Exception as e:
                print(f"Error: {e}")
                break  # Exit loop if connection is lost

# Run WebSocket client
asyncio.run(receive_data())