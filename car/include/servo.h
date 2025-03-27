#ifndef ESP32_SLAM_SERVO_MOTOR_H
#define ESP32_SLAM_SERVO_MOTOR_H

#include <Arduino.h>
#include <ESP32Servo.h>  // ESP32 requires this library for servo control

class ServoMotor {
private:
    Servo servo;
    int servoPin;

public:
    ServoMotor(int pin);
    void begin();
    void setAngle(int angle);
};

#endif
