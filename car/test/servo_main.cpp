#include <Arduino.h>
#include "../include/servo.h"

#define SERVO_PIN 13  // Change this pin based on your ESP32 wiring

ServoMotor servo(SERVO_PIN);

void setup() {
    Serial.begin(115200);
    servo.begin();
}

void loop() {
    for (int angle = 0; angle <= 180; angle += 10) {
        servo.setAngle(angle);
        Serial.print("Servo Angle: ");
        Serial.println(angle);
        delay(500);
    }

    for (int angle = 180; angle >= 0; angle -= 10) {
        servo.setAngle(angle);
        Serial.print("Servo Angle: ");
        Serial.println(angle);
        delay(500);
    }
}
