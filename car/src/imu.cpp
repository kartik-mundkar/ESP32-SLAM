#include "../include/imu.h"


IMUSensor::IMUSensor(int imuPins[]){
    this->roll = 0, this->pitch =0 , this->yaw = 0 ;
    this->accelBias[0] = 0, this->accelBias[1] = 0, this->accelBias[2] = 0;
    this->gyroBias[0] = 0, this->gyroBias[1] = 0, this->gyroBias[2] = 0;
    this->SDA = imuPins[0]; // I2C SDA pin
    this->SCL = imuPins[1]; // I2C SCL pin
    Wire.begin(SDA, SCL); // Initialize I2C with custom SDA and SCL pins
    // Wire.setClock(200000); // Set I2C speed to 200kHz (default is 100kHz)
    Serial.println("IMU Sensor Initialized");
}

void IMUSensor::begin()
{
    I2CwriteByte(MPU9250_ADDRESS, 28, ACC_FULL_SCALE_4_G);
    I2CwriteByte(MPU9250_ADDRESS, 27, GYRO_FULL_SCALE_2000_DPS);
    I2CwriteByte(MPU9250_ADDRESS, 0x37, 0x02);
    // I2CwriteByte(MAG_ADDRESS, 0x0A, 0x01);
    delay(1000);
    calibrateIMU();
}

void IMUSensor::I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t *Data)
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

void IMUSensor::I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data)
{
    Wire.beginTransmission(Address);
    Wire.write(Register);
    Wire.write(Data);
    Wire.endTransmission();
}

void IMUSensor::applyLowPassFilter(IMUData &data)
{
    static float prevAccel[3] = {0, 0, 0};
    // static float prevGyro[3] = {0, 0, 0};

    // Apply LPF
    for (int i = 0; i < 3; i++)
    {
        data.accel[i] = ALPHA * data.accel[i] + (1 - ALPHA) * prevAccel[i];
        // data.gyro[i] = ALPHA * data.gyro[i] + (1 - ALPHA) * prevGyro[i];

        // Update previous values
        prevAccel[i] = data.accel[i];
        // prevGyro[i] = data.gyro[i];
    }
}

void IMUSensor::applyKalmanFilter(IMUData &data)
{
    static float x[3] = {0, 0, 0}; // Estimated state
    static float P[3] = {1, 1, 1}; // Error covariance
    float Q = 0.005;               // Process noise covariance
    float R = 0.02;                // Measurement noise covariance

    for (int i = 0; i < 3; i++)
    {
        float P_pred = P[i] + Q;
        float K = P_pred / (P_pred + R); // Kalman gain
        x[i] = x[i] + K * (data.accel[i] - x[i]);
        P[i] = (1 - K) * P_pred;
        data.accel[i] = x[i];
    }
}

// High-Pass Filter (HPF) implementation
void IMUSensor::applyHighPassFilter(IMUData &data)
{
    static float prevAccel[3] = {0, 0, 0};
    // static float prevGyro[3] = {0, 0, 0};
    static float prevAccelOutput[3] = {0, 0, 0};
    // static float prevGyroOutput[3] = {0, 0, 0};

    float alpha = 0.1; // HPF coefficient, tune this value

    for (int i = 0; i < 3; i++)
    {
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
void IMUSensor::applyMedianFilter(IMUData &data)
{
    static float accelBuffer[5][3] = {0};
    // static float gyroBuffer[5][3] = {0};

    // Shift old values in the buffer (from end to beginning)
    for (int i = 3; i >= 0; i--) { // Shift only the first 4 elements
        for (int j = 0; j < 3; j++) {
            accelBuffer[i + 1][j] = accelBuffer[i][j];
            // gyroBuffer[i + 1][j] = gyroBuffer[i][j];
        }
    }

    // Store new sensor readings in the first position
    for (int i = 0; i < 3; i++) {
        accelBuffer[0][i] = data.accel[i];
        // gyroBuffer[0][i] = data.gyro[i];
    }

    // Apply median filter to accelerometer and gyroscope data
    for (int i = 0; i < 3; i++) {
        data.accel[i] = median(accelBuffer, i); // Find median for accel data
        // data.gyro[i] = median(gyroBuffer, i);   // Find median for gyro data
    }
}

// Helper function to calculate median of 5 values
float IMUSensor::median(float buffer[7][3], int index)
{
    float values[7];
    for (int i = 0; i < 5; i++)
    {
        values[i] = buffer[i][index];
    }
    std::sort(values, values + 5); // Sort the values
    return values[2]; // Return the middle value (the median)
}

void IMUSensor::calibrateIMU() {
    int numSamples = 100; // Increase the number of samples
    float gyroSum[3] = {0}, accelSum[3] = {0};
    uint8_t Buf[14];

    // Discard initial unstable readings
    for (int i = 0; i < 10; i++) {
        I2Cread(MPU9250_ADDRESS, 0x3B, 14, Buf);
        delay(50);
    }

    // Collect samples
    for (int i = 0; i < numSamples; i++) {
        I2Cread(MPU9250_ADDRESS, 0x3B, 14, Buf);

        int16_t axx = -(Buf[0] << 8 | Buf[1]);
        int16_t ayy = -(Buf[2] << 8 | Buf[3]);
        int16_t azz = (Buf[4] << 8 | Buf[5]);
        int16_t gxx = (Buf[8] << 8 | Buf[9]);
        int16_t gyy = (Buf[10] << 8 | Buf[11]);
        int16_t gzz = -(Buf[12] << 8 | Buf[13]);

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

        delay(50); // Increase delay for stability
    }

    // Calculate biases
    for (int i = 0; i < 3; i++) {
        gyroBias[i] = gyroSum[i] / numSamples;
        accelBias[i] = accelSum[i] / numSamples;
    }

    // Adjust Z-axis accelerometer bias for gravity
    accelBias[2] -= g;

    // Print biases for debugging
    // Serial.printf("Accel Bias: X=%.2f, Y=%.2f, Z=%.2f\n", accelBias[0], accelBias[1], accelBias[2]);
    // Serial.printf("Gyro Bias: X=%.2f, Y=%.2f, Z=%.2f\n", gyroBias[0], gyroBias[1], gyroBias[2]);
}

IMUData IMUSensor::readSensor(float dt)
{
    uint8_t Buf[14];
    I2Cread(MPU9250_ADDRESS, 0x3B, 14, Buf);

    IMUData data;
    int16_t ax = -(Buf[0] << 8 | Buf[1]);
    int16_t ay = -(Buf[2] << 8 | Buf[3]);
    int16_t az = (Buf[4] << 8 | Buf[5]);
    int16_t gx = (Buf[8] << 8 | Buf[9]);
    int16_t gy = (Buf[10] << 8 | Buf[11]);
    int16_t gz = -(Buf[12] << 8 | Buf[13]);

    data.accel[0] = ax * g / acc_sensitivity - accelBias[0];
    data.accel[1] = ay * g / acc_sensitivity - accelBias[1];
    data.accel[2] = az * g / acc_sensitivity - accelBias[2];

    gx = (gx / 16.4 - gyroBias[0]) * dt;
    gy = (gy / 16.4 - gyroBias[1]) * dt;
    gz = (gz / 16.4 - gyroBias[2]) * dt;

    data.gyro[0] = gx;
    data.gyro[1] = gy;
    data.gyro[2] = gz;

    applyMedianFilter(data);   // Apply Median Filter
    applyLowPassFilter(data);
    applyHighPassFilter(data); // Apply High-Pass Filter
    applyKalmanFilter(data);

    roll += round(data.gyro[0] * 100) / 100;
    pitch += round(data.gyro[1] * 100) / 100;
    yaw += round(data.gyro[2] * 100) / 100;

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

String IMUSensor::getIMUJson(IMUData data) {
    return "{\"aX\":" + String(data.accel[0], 2) +
           ",\"aY\":" + String(data.accel[1], 2) +
           ",\"aZ\":" + String(data.accel[2], 2) +
           ",\"gX\":" + String(data.gyro[0], 2) +
           ",\"gY\":" + String(data.gyro[1], 2) +
           ",\"gZ\":" + String(data.gyro[2], 2) +
           ",\"R\":" + String(data.roll, 2) +
           ",\"P\":" + String(data.pitch, 2) +
           ",\"Y\":" + String(data.yaw, 2) + "}";
}