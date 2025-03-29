#include "../include/imu.h"
#include "config.h"

// Constructor
IMUSensor::IMUSensor() : accAngleX(0), accAngleY(0), gyroAngleX(0), gyroAngleY(0), prevTime(0) {}

// Initialize MPU9250 sensor
void IMUSensor::begin() {
    Wire.begin(SDA_PIN, SCL_PIN, I2C_CLOCK_SPEED);
    mpu.setWire(&Wire);  // Set I2C wire instance for MPU9250_asukiaaa library

    uint8_t sensorId;
    int retryCount = 0;
    const int maxRetries = 5;  // Maximum attempts to connect

    while (mpu.readId(&sensorId) != 0 && retryCount < maxRetries) {
        Serial.print("IMU Sensor not found! Retrying... (Attempt ");
        Serial.print(retryCount + 1);
        Serial.println(")");

        delay(1000); // Delay between retry attempts
        retryCount++;
    }

    if (retryCount == maxRetries) {
        Serial.println("IMU Sensor initialization failed after multiple attempts.");
        return;  // Continue the program without IMU functionality
    }

    // Proceed with initialization if connection is successful
    mpu.beginAccel(ACC_FULL_SCALE_8_G);
    mpu.beginGyro(GYRO_FULL_SCALE_500_DPS);
    mpu.beginMag(MAG_MODE_CONTINUOUS_100HZ);

    // Magnetometer Calibration (Example values, adjust as needed)
    mpu.magXOffset = -20;
    mpu.magYOffset = 15;
    mpu.magZOffset = 30;
    
    Serial.println("IMU Sensor initialized successfully.");
}

// Read acceleration data
void IMUSensor::readAcceleration(float &ax, float &ay, float &az) {
    mpu.accelUpdate();
    ax = mpu.accelX();
    ay = mpu.accelY();
    az = mpu.accelZ();
}

// Read gyroscope data
void IMUSensor::readGyroscope(float &gx, float &gy, float &gz) {
    mpu.gyroUpdate();
    gx = mpu.gyroX();
    gy = mpu.gyroY();
    gz = mpu.gyroZ();
}

// Read magnetometer data
void IMUSensor::readMagnetometer(float &mx, float &my, float &mz) {
    mpu.magUpdate();
    mx = mpu.magX();
    my = mpu.magY();
    mz = mpu.magZ();
}

// Read all IMU data
void IMUSensor::readIMUData(float &ax, float &ay, float &az, float &gx, float &gy, float &gz, float &mx, float &my, float &mz) {
    readAcceleration(ax, ay, az);
    readGyroscope(gx, gy, gz);
    readMagnetometer(mx, my, mz);
}

// Estimate orientation using complementary filter
void IMUSensor::estimateOrientation(float &roll, float &pitch, float &yaw) {
    float ax, ay, az, gx, gy, gz, mx, my, mz;
    readIMUData(ax, ay, az, gx, gy, gz, mx, my, mz);

    // Time difference calculation for gyro integration
    unsigned long currTime = millis();
    float dt = (currTime >= prevTime) ? (currTime - prevTime) / 1000.0 : (currTime + (ULONG_MAX - prevTime)) / 1000.0;
    prevTime = currTime;

    // Angle estimation from accelerometer (tilt angle)
    accAngleX = atan2(ay, sqrt(ax * ax + az * az)) * RAD_TO_DEG;
    accAngleY = atan2(-ax, sqrt(ay * ay + az * az)) * RAD_TO_DEG;

    // Angle estimation from gyroscope
    gyroAngleX += gx * dt;
    gyroAngleY += gy * dt;
    gyroAngleZ += gz * dt;

    // Magnetometer-based yaw estimation
    float magYaw = atan2(my, mx) * RAD_TO_DEG; // Heading angle from magnetometer

    // Complementary filter for smooth orientation
    roll  = 0.96 * gyroAngleX + 0.04 * accAngleX;
    pitch = 0.96 * gyroAngleY + 0.04 * accAngleY;
    // yaw   = 0.96 * gyroAngleZ + 0.04 * magYaw; // Correcting yaw drift using magnetometer
    // If magnetometer is not available, use gyro for yaw estimation
    yaw = gyroAngleZ;
}
