#include "IMU.h"

IMU::IMU() : roll(0), pitch(0), yaw(0) {}

void IMU::begin()
{
    Wire.begin(32, 33);
    I2CwriteByte(MPU9250_ADDRESS, 28, ACC_FULL_SCALE_16_G);
    I2CwriteByte(MPU9250_ADDRESS, 27, GYRO_FULL_SCALE_2000_DPS);
    I2CwriteByte(MPU9250_ADDRESS, 0x37, 0x02);
    I2CwriteByte(MAG_ADDRESS, 0x0A, 0x01);

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

void IMU::calibrate()
{
    int numSamples = 200;
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

        float ax = axx * 9.81 / 2048;
        float ay = ayy * 9.81 / 2048;
        float az = azz * 9.81 / 2048;
        float gx = gxx / 16.4;
        float gy = gyy / 16.4;
        float gz = gzz / 16.4;

        accelSum[0] += ax;
        accelSum[1] += ay;
        accelSum[2] += az;
        gyroSum[0] += gx;
        gyroSum[1] += gy;
        gyroSum[2] += gz;

        delay(10);
    }

    for (int i = 0; i < 3; i++)
    {
        gyroBias[i] = gyroSum[i] / numSamples;
        accelBias[i] = accelSum[i] / numSamples;
    }

    accelBias[2] -= 9.81;
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

    data.accel[0] = ax * 9.81 / 2048 - accelBias[0];
    data.accel[1] = ay * 9.81 / 2048 - accelBias[1];
    data.accel[2] = az * 9.81 / 2048 - accelBias[2];

    gx = (gx / 16.4 - gyroBias[0])*dt;
    gy = (gy / 16.4 - gyroBias[1])*dt;
    gz = (gz / 16.4 - gyroBias[2])*dt;

    data.gyro[0] = gx;
    data.gyro[1] = gy;
    data.gyro[2] = gz;

    roll += data.gyro[0];
    pitch += data.gyro[1];
    yaw += data.gyro[2] ;

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
