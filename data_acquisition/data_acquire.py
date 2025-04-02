import asyncio
import websockets
import json

# Replace with your ESP32's IP Address
ESP32_WS_URL = "ws://192.168.137.120:81"

clean_data_store_file = "mov.csv"
log_data_store_file = "rest10.log"
async def receive_data():
    async with websockets.connect(ESP32_WS_URL) as websocket:
        print("Connected to ESP32 WebSocket!")
                # ✅ Wait for ESP32 acknowledgment
        ack_message = await websocket.recv()
        print(f"ESP32 says: {ack_message}")

        with open(clean_data_store_file,'w') as csvFile:
            csvFile.write('time,ax,ay,az,gx,gy,gz,roll,pitch,yaw\n')

        while True:
            try:
                # Receive WebSocket message
                message = await websocket.recv()
                data = json.loads(message)  # Convert JSON string to dictionary
                # print(data)
                time = data["timestamp"]
                imu = data["imu"]
                string = ','.join(list(map(str,imu.values())))
                with open(clean_data_store_file,'a+') as csvFile:
                    csvFile.write(str(time)+","+string+"\n")

                with open(log_data_store_file,'+a') as jsonFile:
                    jsonFile.write(json.dumps(data))
                    jsonFile.write('\n')


            except Exception as e:
                print(f"Error: {e}")
                break  # Exit loop if connection is lost

# Run WebSocket client
asyncio.run(receive_data())
