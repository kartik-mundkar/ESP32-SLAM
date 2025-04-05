#ifndef ESP32_SLAM_CAR_H
#define ESP32_SLAM_CAR_H

#include <Arduino.h>
#include "motor.h"
#include "servo.h"
#include "imu.h"
#include "ultrasonic_sensor.h"

const int MIN_DISTANCE = 20;

class Car : public Motor, public IMUSensor, public UltrasonicSensor {
private:
    int speed;

public:
    Car(int motorPins[], int imuPins[], int ultrasonicPins[], int servoPin);
    ~Car();
    void initComponents();
    void drive(char command);
    bool isObstacleDetected();
    UltraData ultraScan();
    String getCarData(float dt);
};

#endif // ESP32_SLAM_CAR_H
