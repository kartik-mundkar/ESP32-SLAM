#include <Arduino.h>
#include <WiFi.h>
#include "../include/websocket_imu.h"
#include "MPU9250_asukiaaa.h"
#include "../include/json_utility.h"

#define WIFI_SSID "YourWiFiSSID"
#define WIFI_PASSWORD "YourWiFiPassword"

MPU9250_asukiaaa mpu;
WebSocketIMU webSocketIMU(81);

void setup() {
    Serial.begin(115200);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWiFi connected.");

    // MPU Initialization
    Wire.begin();
    mpu.setWire(&Wire);
    Serial.println("MPU9250 initialized successfully.");
    webSocketIMU.begin();
}

void loop() {
    webSocketIMU.loop();

    // IMU Data Retrieval
    mpu.accelUpdate();
    mpu.gyroUpdate();
    mpu.magUpdate();

    // Read Sensor Data
    float ax = mpu.accelX();
    float ay = mpu.accelY();
    float az = mpu.accelZ();

    float gx = mpu.gyroX();
    float gy = mpu.gyroY();
    float gz = mpu.gyroZ();

    float mx = mpu.magX();
    float my = mpu.magY();
    float mz = mpu.magZ();

    // Create Flexible JSON Data with Only Needed Values
    String imuData = JsonUtility::createJson(
        {{"sensor", "MPU9250"}, {"status", "active"}},
        {{"signalStrength", 87}},
        {{"accelX", ax}, {"accelY", ay}, {"accelZ", az},
         {"gyroX", gx}, {"gyroY", gy}, {"gyroZ", gz},
         {"magX", mx}, {"magY", my}, {"magZ", mz}},
        {{"connected", true}}
    );

    webSocketIMU.broadcastData(imuData);
    Serial.println("JSON Data Sent: " + imuData);

    delay(100);  // Control data frequency
}
