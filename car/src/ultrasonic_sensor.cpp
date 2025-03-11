#include "../include/ultrasonic_sensor.h"

UltrasonicSensor::UltrasonicSensor(int trigPin, int echoPin) {
    this->triggerPin = trigPin;
    this->echoPin = echoPin;
}

void UltrasonicSensor::begin() {
    pinMode(triggerPin, OUTPUT);
    pinMode(echoPin, INPUT);
}

float UltrasonicSensor::getDistance() {
    digitalWrite(triggerPin, LOW);
    delayMicroseconds(2);
    digitalWrite(triggerPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(triggerPin, LOW);

    long duration = pulseIn(echoPin, HIGH);
    float distance = (duration * 0.0343) / 2;  // Convert time to cm
    return distance;
}
