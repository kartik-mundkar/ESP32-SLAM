#include <Arduino.h>
#include "..\include\imu.h"

IMU imu;
unsigned long lastTime = 0;

void setup() {
    Serial.begin(115200);
    imu.begin();
    Serial.println("started IMU");
}

void loop() {
    unsigned long currentTime = millis();
    float dt = (currentTime - lastTime) / 1000.0;
    lastTime = currentTime;

    IMUData data = imu.readSensor(dt); // No arguments needed!

    Serial.print("Ax: "); Serial.print(data.accel[0]); Serial.print("\t");
    Serial.print("Ay: "); Serial.print(data.accel[1]); Serial.print("\t");
    Serial.print("Az: "); Serial.print(data.accel[2]); Serial.print("\t");
    Serial.print("Gx: "); Serial.print(data.gyro[0]); Serial.print("\t");
    Serial.print("Gy: "); Serial.print(data.gyro[1]); Serial.print("\t");
    Serial.print("Gz: "); Serial.print(data.gyro[2]); Serial.print("\t");
    Serial.print("Roll: "); Serial.print(data.roll); Serial.print("\t");
    Serial.print("Pitch: "); Serial.print(data.pitch); Serial.print("\t");
    Serial.print("Yaw: "); Serial.println(data.yaw); Serial.print("\n");

    delay(50);
}
