#include "../include/servo.h"

ServoMotor::ServoMotor(int pin) {
    this->servoPin = pin;
}

void ServoMotor::begin() {
    servo.attach(servoPin);
}

void ServoMotor::setAngle(int angle) {
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;
    servo.write(angle);
}
