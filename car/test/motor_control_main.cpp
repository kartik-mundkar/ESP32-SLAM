#include <Arduino.h>
#include "../include/motor_control.h"

void setup() {
    Serial.begin(115200);
    setupMotors();
}

void loop() {
    Serial.println("Moving Forward");
    moveForward(150);  // Speed 0-255
    delay(2000);

    Serial.println("Turning Left");
    turnLeft(150);
    delay(1000);

    Serial.println("Turning Right");
    turnRight(150);
    delay(1000);

    Serial.println("Moving Backward");
    moveBackward(150);
    delay(2000);

    Serial.println("Stopping");
    stopCar();
    delay(2000);
}
