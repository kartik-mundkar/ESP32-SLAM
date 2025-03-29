#include "../include/ultrasonic_sensor.h"
#include "../include/json_utility.h"

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

String UltrasonicSensor::get180Scan() {
    String scanData = "";
    for (int i = 0; i < 180; i+=10) {
        scanData += JsonUtility::appendJson(scanData, {{"angle", i}, {"distance", getDistance()}});
        delay(500);
    }
    Serial.println(scanData);
    return scanData;
}
