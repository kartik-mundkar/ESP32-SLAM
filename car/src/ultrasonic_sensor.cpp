#include "../include/ultrasonic_sensor.h"


UltrasonicSensor::UltrasonicSensor(int trigPin, int echoPin, int servoPin):ServoMotor(servoPin) {
    this->triggerPin = trigPin;
    this->echoPin = echoPin;
    Serial.println("Ultrasonic Sensor Initialized");
}

void UltrasonicSensor::begin() {
    pinMode(triggerPin, OUTPUT);
    pinMode(echoPin, INPUT);
    ServoMotor::begin();
    ServoMotor::setAngle(90); // Set initial position to 90 degrees
}

float UltrasonicSensor::getDistance() {
    digitalWrite(triggerPin, LOW);
    delayMicroseconds(2);
    digitalWrite(triggerPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(triggerPin, LOW);

    long duration = pulseIn(echoPin, HIGH);
    float distance = (duration * 0.0343) / 2;  // Convert time to cm
    
    return (distance > MAX_RANGE) ? -1 : distance;  // Return -1 if out of range
}

// Function to perform a complete scan
UltraData UltrasonicSensor::performScan() {
    scanSize = 0;
    for (int angle = SCAN_START; angle <= SCAN_END; angle += STEP_SIZE) {
        ServoMotor::setAngle(angle);
        delay(20);  // Wait for the servo to stabilize

        ultraData.angles[scanSize] = angle;
        ultraData.distances[scanSize] = getDistance();
        scanSize++;
    }
    ServoMotor::setAngle(90);
    ultraData.size = scanSize;
    return ultraData;
}


// Function to generate JSON for ultrasonic scan
String UltrasonicSensor::getJson(int angles[], float distances[], int size) {
    String json = "{";
    json += "\"angle\":[";
    for (int i = 0; i < size; i++) {
        json += String(angles[i]);
        if (i < size - 1) json += ",";
    }
    json += "],\"distance\":[";
    for (int i = 0; i < size; i++) {
        json += String(distances[i], 2);
        if (i < size - 1) json += ",";
    }
    json += "]}";
    return json;
}
