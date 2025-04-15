#include "../include/servo.h"

ServoMotor::ServoMotor(int pin) {
    this->servoPin = pin;
}

void ServoMotor::begin() {
    servo.attach(servoPin);
    // lets show that servo is working by taking 0 to 180 degree sweep
    for (int pos = 0; pos <= 180; pos += 2) { // goes from 0 degrees to 180 degrees
        servo.write(pos);              // tell servo to go to position in variable 'pos'
        delay(20);                     // waits 20ms for the servo to reach the position
    }
    servo.write(90); // Set initial position to 90 degrees
    delay(100); // Wait for 1 second to stabilize
}

void ServoMotor::setAngle(int angle) {
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;
    servo.write(angle);
}
