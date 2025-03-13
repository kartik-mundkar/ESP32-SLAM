#include <Arduino.h>
#include "../include/ultrasonic_sensor.h"

// Define pins for ESP32
#define TRIGGER_PIN 26
#define ECHO_PIN 27

UltrasonicSensor sensor(TRIGGER_PIN, ECHO_PIN);

void setup() {
    Serial.begin(115200);
    sensor.begin();
}

void loop() {
    float distance = sensor.getDistance();
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" cm");
    delay(500);
}
