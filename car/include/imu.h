#ifndef ESP32_SLAM_IMU_SENSOR_H
#define ESP32_SLAM_IMU_SENSOR_H

#include <Arduino.h>
#include <Wire.h>
#include <MPU9250_asukiaaa.h> // Library for MPU-9250/6500 IMU sensor

class IMUSensor {
public:
    IMUSensor();

    // Initialization method
    void begin();

    // Data reading methods
    void readAcceleration(float &ax, float &ay, float &az);
    void readGyroscope(float &gx, float &gy, float &gz);
    void readMagnetometer(float &mx, float &my, float &mz); // For MPU-9250 only

    // Combined method for all data
    void readIMUData(float &ax, float &ay, float &az, float &gx, float &gy, float &gz, float &mx, float &my, float &mz);

    // Orientation estimation using complementary filter
    void estimateOrientation(float &roll, float &pitch, float &yaw);

private:
    MPU9250_asukiaaa mpu;
    float accAngleX, accAngleY;
    float gyroAngleX, gyroAngleY, gyroAngleZ;
    float prevTime;
};

#endif // IMU_SENSOR_H
