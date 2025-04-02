#include "../include/imu.h"


IMU::IMU() : roll(0), pitch(0), yaw(0) {}

void IMU::begin()
{
    Wire.begin(32, 33);
    Wire.setClock(200000);  // Set I2C speed to 200kHz (default is 100kHz)
    I2CwriteByte(MPU9250_ADDRESS, 28, ACC_FULL_SCALE_4_G);
    I2CwriteByte(MPU9250_ADDRESS, 27, GYRO_FULL_SCALE_2000_DPS);
    I2CwriteByte(MPU9250_ADDRESS, 0x37, 0x02);
    // I2CwriteByte(MAG_ADDRESS, 0x0A, 0x01);
    delay(1000);
    calibrate();
}

void IMU::I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t *Data)
{
    Wire.beginTransmission(Address);
    Wire.write(Register);
    Wire.endTransmission();
    Wire.requestFrom(Address, Nbytes);
    for (uint8_t i = 0; i < Nbytes; i++)
    {
        if (Wire.available())
            Data[i] = Wire.read();
    }
}

void IMU::I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data)
{
    Wire.beginTransmission(Address);
    Wire.write(Register);
    Wire.write(Data);
    Wire.endTransmission();
}


void IMU::applyLowPassFilter(IMUData &data) {
    static float prevAccel[3] = {0, 0, 0};
    static float prevGyro[3] = {0, 0, 0};

    // Apply LPF
    for (int i = 0; i < 3; i++) {
        data.accel[i] = ALPHA * data.accel[i] + (1 - ALPHA) * prevAccel[i];
        data.gyro[i] = ALPHA * data.gyro[i] + (1 - ALPHA) * prevGyro[i];

        // Update previous values
        prevAccel[i] = data.accel[i];
        prevGyro[i] = data.gyro[i];
    }
}

void IMU::applyKalmanFilter(IMUData &data) {
    static float x[3] = {0, 0, 0}; // Estimated state
    static float P[3] = {1, 1, 1}; // Error covariance
    float Q = 0.001; // Process noise covariance
    float R = 0.01;  // Measurement noise covariance

    for (int i = 0; i < 3; i++) {
        float P_pred = P[i] + Q;
        float K = P_pred / (P_pred + R); // Kalman gain
        x[i] = x[i] + K * (data.accel[i] - x[i]);
        P[i] = (1 - K) * P_pred;
        data.accel[i] = x[i];
    }
}

// High-Pass Filter (HPF) implementation
void IMU::applyHighPassFilter(IMUData &data) {
    static float prevAccel[3] = {0, 0, 0};
    // static float prevGyro[3] = {0, 0, 0};
    static float prevAccelOutput[3] = {0, 0, 0};
    // static float prevGyroOutput[3] = {0, 0, 0};
    
    float alpha = 0.1; // HPF coefficient, tune this value

    for (int i = 0; i < 3; i++) {
        // High-Pass filter for accelerometer and gyroscope
        data.accel[i] = alpha * (prevAccelOutput[i] + data.accel[i] - prevAccel[i]);
        // data.gyro[i] = alpha * (prevGyroOutput[i] + data.gyro[i] - prevGyro[i]);

        // Save the current data for the next loop
        prevAccel[i] = data.accel[i];
        // prevGyro[i] = data.gyro[i];
        prevAccelOutput[i] = data.accel[i];
        // prevGyroOutput[i] = data.gyro[i];
    }
}

// Median Filter implementation for removing spikes/outliers
void IMU::applyMedianFilter(IMUData &data) {
    static float accelBuffer[5][3] = {0};
    static float gyroBuffer[5][3] = {0};

    // Store new sensor readings in buffer
    for (int i = 0; i < 3; i++) {
        accelBuffer[4][i] = data.accel[i];
        gyroBuffer[4][i] = data.gyro[i];
    }

    // Shift old values in the buffer
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 3; j++) {
            accelBuffer[i][j] = accelBuffer[i + 1][j];
            gyroBuffer[i][j] = gyroBuffer[i + 1][j];
        }
    }

    // Apply median filter to accelerometer and gyroscope data
    for (int i = 0; i < 3; i++) {
        data.accel[i] = median(accelBuffer, i);  // Find median for accel data
        data.gyro[i] = median(gyroBuffer, i);    // Find median for gyro data
    }
}

// Helper function to calculate median of 5 values
float IMU::median(float buffer[5][3], int index) {
    float values[5];
    for (int i = 0; i < 5; i++) {
        values[i] = buffer[i][index];
    }
    std::sort(values, values + 5);
    return values[2]; // Return the middle value (the median)
}

void IMU::calibrate()
{
    int numSamples = 500;
    float gyroSum[3] = {0}, accelSum[3] = {0};

    uint8_t Buf[14];
    I2Cread(MPU9250_ADDRESS, 0x3B, 14, Buf);
    for (int i = 0; i < numSamples; i++)
    {

        int16_t axx = -(Buf[0] << 8 | Buf[1]);
        int16_t ayy = -(Buf[2] << 8 | Buf[3]);
        int16_t azz = Buf[4] << 8 | Buf[5];
        int16_t gxx = -(Buf[8] << 8 | Buf[9]);
        int16_t gyy = -(Buf[10] << 8 | Buf[11]);
        int16_t gzz = Buf[12] << 8 | Buf[13];

        float ax = axx * g / acc_sensitivity;
        float ay = ayy * g / acc_sensitivity;
        float az = azz * g / acc_sensitivity;
        float gx = gxx / gyro_sensitivity;
        float gy = gyy / gyro_sensitivity;
        float gz = gzz / gyro_sensitivity;

        accelSum[0] += ax;
        accelSum[1] += ay;
        accelSum[2] += az;
        gyroSum[0] += gx;
        gyroSum[1] += gy;
        gyroSum[2] += gz;

        delay(20);
    }

    for (int i = 0; i < 3; i++)
    {
        gyroBias[i] = gyroSum[i] / numSamples;
        accelBias[i] = accelSum[i] / numSamples;
    }

    accelBias[2] -= g;
}

IMUData IMU::readSensor(float dt)
{
    uint8_t Buf[14];
    I2Cread(MPU9250_ADDRESS, 0x3B, 14, Buf);

    IMUData data;
    int16_t ax = -(Buf[0] << 8 | Buf[1]);
    int16_t ay = -(Buf[2] << 8 | Buf[3]);
    int16_t az = Buf[4] << 8 | Buf[5];
    int16_t gx = -(Buf[8] << 8 | Buf[9]);
    int16_t gy = -(Buf[10] << 8 | Buf[11]);
    int16_t gz = Buf[12] << 8 | Buf[13];

    data.accel[0] = ax * g / acc_sensitivity - accelBias[0];
    data.accel[1] = ay * g / acc_sensitivity - accelBias[1];
    data.accel[2] = az * g / acc_sensitivity - accelBias[2];

    gx = (gx / 16.4 - gyroBias[0])*dt;
    gy = (gy / 16.4 - gyroBias[1])*dt;
    gz = (gz / 16.4 - gyroBias[2])*dt;

    data.gyro[0] = gx;
    data.gyro[1] = gy;
    data.gyro[2] = gz;
    
    applyLowPassFilter(data);
    applyKalmanFilter(data);
    applyHighPassFilter(data); // Apply High-Pass Filter
    applyMedianFilter(data);   // Apply Median Filter

    roll += round(data.gyro[0]*100)/100;
    pitch += round(data.gyro[1]*100)/100;
    yaw += round(data.gyro[2]*100)/100 ;

    if (abs(roll) > 360)
        roll = 0;
    if (abs(pitch) > 360)
        pitch = 0;
    if (abs(yaw) > 360)
        yaw = 0;

    data.roll = roll;
    data.pitch = pitch;
    data.yaw = yaw;

    return data;
}
