#ifndef ESP32_SLAM_IMU_SENSOR_H
#define ESP32_SLAM_IMU_SENSOR_H

#include <Wire.h>
#include <math.h>

#define MPU9250_ADDRESS 0x68
#define MAG_ADDRESS 0x0C
#define GYRO_FULL_SCALE_2000_DPS 0x18
#define ACC_FULL_SCALE_16_G 0x18

struct IMUData {
    float accel[3];  // {ax, ay, az} in m/s²
    float gyro[3];   // {gx, gy, gz} in degrees/s
    float roll, pitch, yaw; // Orientation angles in degrees
};

class IMU {
public:
    IMU();
    void begin();
    void calibrate();
    IMUData readSensor(float dt);
    
private:
    void I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data);
    void I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data);
    
    float gyroBias[3] = {0};  // Bias correction for gyroscope
    float accelBias[3] = {0}; // Bias correction for accelerometer
    float roll, pitch, yaw;  // Stores ongoing roll, pitch, yaw values
};

#endif
