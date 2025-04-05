// #include <Arduino.h>
// #include <WiFi.h>
// #include <WebSocketsServer.h>
// #include <Wire.h>
// #include "../include/imu.h"                 // Include IMU header file
// #include "../include/ultrasonic_sensor.h"                 // Include IMU header file

// // #include <utils.h>
// // WiFi Credentials
// const char* ssid = "KARTIK 2824";
// const char* password = "84Q38#3u";

// // MPU6050 Sensor
// IMUSensor imu;  // Create an instance of IMUSensor
// // UltrasonicSensor sensor(TRIGGER_PIN, ECHO_PIN);
// UltrasonicSensor ultrasonic;

// float accelX, accelY, accelZ, gyroX, gyroY, gyroZ, magX, magY, magZ;

// //ip
// IPAddress local_IP(192, 168, 137, 120); // Your desired static IP
// IPAddress gateway(192, 168, 1, 1);     // Your router's IP address
// IPAddress subnet(255, 255, 0, 0);     // Your subnet mask
// IPAddress primaryDNS(8, 8, 8, 8);      // Optional: Primary DNS server
// IPAddress secondaryDNS(8, 8, 4, 4);    // Optional: Secondary DNS server

// unsigned long lastTime = 0;
// // WebSocket Server
// WebSocketsServer webSocket(81);

// void sendIMUData(float );
// void sendUltraData();
// void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length);

// void setup_wifi() {
//     WiFi.begin(ssid, password);
//     Serial.print("Connecting to WiFi...");
//     while (WiFi.status() != WL_CONNECTED) {
//         delay(500);
//         Serial.print(".");
//     }
//       // Configure static IP address
//       if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS)) {
//         Serial.println("STA Failed to configure");
//     }

//     Serial.println("\nConnected to WiFi!");
//     Serial.print("IP Address: ");
//     Serial.println(WiFi.localIP());
// }

// void setup() {
//     Serial.begin(115200);
//     setup_wifi();

//     webSocket.begin();
//     webSocket.onEvent(webSocketEvent);


//     imu.begin();
// }

// void loop() {
//   unsigned long currentTime = millis();
//   float dt = (currentTime - lastTime) / 1000.0;
//   lastTime = currentTime;
//     webSocket.loop();
//     // sendIMUData(dt);
//     sendUltraData();
//     delay(50);
// }

// void sendUltraData() {
//     UltraData sensordata = ultrasonic.performScan();
//     int size = sizeof(sensordata.angles)/sizeof(sensordata.angles[0]);
//     String data = ultrasonic.getJson(sensordata.angles,sensordata.distances,size);
//     webSocket.broadcastTXT(data);
// }

// void sendIMUData(float dt) {
//     IMUData sensordata = imu.readSensor(dt);


//     String data = "{";
//     data += "\"mode\":\"moving\",";
//     data += "\"timestamp\":" + String(millis()) + ",";
//     data += "\"imu\":{";
//     data += "\"accelX\":" + String(sensordata.accel[0], 1) + ",";
//     data += "\"accelY\":" + String(sensordata.accel[1], 1) + ",";
//     data += "\"accelZ\":" + String(sensordata.accel[2], 1) + ",";
//     data += "\"gyroX\":" + String(sensordata.gyro[0], 2) + ",";
//     data += "\"gyroY\":" + String(sensordata.gyro[1], 2) + ",";
//     data += "\"gyroZ\":" + String(sensordata.gyro[2], 2) + ",";
//     data += "\"roll\":" + String(sensordata.roll, 1) + ",";
//     data += "\"pitch\":" + String(sensordata.pitch, 1) + ",";
//     data += "\"yaw\":" + String(sensordata.yaw, 1);
//     data += "}}";

//     webSocket.broadcastTXT(data);
// }


// void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
//     if (type == WStype_CONNECTED) {
//       Serial.printf("Client [%u] connected!\n", num);
      
//       // ✅ Send acknowledgment message to Python client
//       webSocket.sendTXT(num, "Connection Established");
//     }
  
//     else if (type == WStype_TEXT) {
//       Serial.printf("Received message: %s\n", payload);
  
//       // ✅ Optionally, echo the message back
//       webSocket.sendTXT(num, payload);
//     }
//   }
  

