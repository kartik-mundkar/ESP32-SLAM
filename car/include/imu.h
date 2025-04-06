#ifndef ESP32_SLAM_IMU_SENSOR_H
#define ESP32_SLAM_IMU_SENSOR_H

#include <Wire.h>
#include <Arduino.h>
#include <math.h>
#include <algorithm>

#define MPU9250_ADDRESS 0x68
#define GYRO_FULL_SCALE_2000_DPS 0x18
#define ACC_FULL_SCALE_4_G       0x08
#define acc_sensitivity 8192 // LSB/g
#define gyro_sensitivity 16.4 

#define g 9.81

// #define MAG_ADDRESS 0x0C
// #define ACC_FULL_SCALE_2_G       0x00
// #define ACC_FULL_SCALE_8_G       0x10
// #define ACC_FULL_SCALE_16_G      0x18


#define ALPHA 0.2  // Smoothing factor (adjust between 0.05 - 0.2 for best results)

struct IMUData {
    float accel[3];  // {ax, ay, az} in m/s²
    float gyro[3];   // {gx, gy, gz} in degrees/s
    float roll, pitch, yaw; // Orientation angles in degrees
};

class IMUSensor {
    public:
    IMUSensor(int imuPins[]);
    void begin();
    void calibrateIMU();
    IMUData readSensor(float dt);
    String getIMUJson(IMUData data); 

    private:
    int SDA , SCL ; // I2C SCL pin
    void I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data);
    void I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data);
    void applyLowPassFilter(IMUData &data);
    void applyKalmanFilter(IMUData &data);
    void applyHighPassFilter(IMUData &data);
    void applyMedianFilter(IMUData &data);
    float median(float buffer[5][3], int index);
    
    float gyroBias[3] = {0};  // Bias correction for gyroscope
    float accelBias[3] = {0}; // Bias correction for accelerometer
    float roll, pitch, yaw;  // Stores ongoing roll, pitch, yaw values
};

#endif
