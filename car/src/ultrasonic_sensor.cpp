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
    float distance = (duration * 0.0343) / 200;  // Convert time to m
    return distance;  // Return distance in meters

    // return (distance > MAX_RANGE) ? -1 : distance;  // Return -1 if out of range
}

// // Function to perform a complete scan
UltraData UltrasonicSensor::performScan() {
    ultraData.size = 0; // Initialize size to 0

    // Initialize angles and distances arrays
    for(int i = 0; i < MAX_SCANS; i++) {
        ultraData.angles[i] = 0;
        ultraData.distances[i] = 0.0;
    }
    Serial.println("Performing scan...");
    
    float angle = 0.0;
    for(int i=0; i<num_of_iterations ;i++) {
        scanSize = 0;
        for (int j = 0; j < MAX_SCANS; j ++) {
            angle = j * STEP_SIZE + SCAN_START; // Calculate the current angle
            ServoMotor::setAngle(angle); // Move servo to the current angle
            delay(20);  // Wait for the servo to stabilize

            ultraData.angles[j] = angle;
            ultraData.distances[j] += getDistance()/num_of_iterations; // Average distance
            scanSize++;
        }
        delay(250); // Wait for 1 second before each scan
        ServoMotor::setAngle(90);
        ultraData.size = scanSize;
        delay(250); // Wait for 1 second before each scan
    }

    // Adjust distances that exceed MAX_RANGE
    for (int i = 0; i < MAX_SCANS; i++) {
        if (ultraData.distances[i] > MAX_RANGE) {
            // Handle edge cases for the first and last elements
            if (i == 0) {
                // If it's the first element, use the next angle's distance
                if (ultraData.distances[i + 1] > MAX_RANGE) {
                    ultraData.distances[i] = MAX_RANGE;
                } else {
                    ultraData.distances[i] = ultraData.distances[i + 1];
                }
            } else if (i == MAX_SCANS - 1) {
                // If it's the last element, use the previous angle's distance
                ultraData.distances[i] = ultraData.distances[i - 1];
            } else {
                // Otherwise, calculate the average of the previous and next distances
                if (ultraData.distances[i + 1] > MAX_RANGE) {
                    ultraData.distances[i] = MAX_RANGE;
                } else {
                    ultraData.distances[i] = (ultraData.distances[i - 1] + ultraData.distances[i + 1]) / 2.0;
                }
            }
        }
    }
    
    return ultraData;
}


UltraData UltrasonicSensor::performScanStep() {
    static int currentAngle = SCAN_START; // Start angle
    static unsigned long lastStepTime = 0; // Timestamp for the last step
    static bool isScanning = false; // Scanning state

    if (!isScanning) {
        // Initialize scan
        scanSize = 0;
        currentAngle = SCAN_START;
        isScanning = true;
        lastStepTime = millis();
        ServoMotor::setAngle(currentAngle); // Move servo to the starting angle
        return ultraData; // Return empty data initially
    }

    // Check if enough time has passed to move to the next step
    if (millis() - lastStepTime >= SCAN_DELAY) { // 20 ms for servo stabilization
        // Record the distance for the current angle
        ultraData.angles[scanSize] = currentAngle;
        ultraData.distances[scanSize] = getDistance();
        scanSize++;

        // Move to the next angle
        currentAngle += STEP_SIZE;
        if (currentAngle <= SCAN_END) {
            ServoMotor::setAngle(currentAngle); // Move servo to the next angle
            lastStepTime = millis(); // Update the timestamp
        } else {
            // Scan complete
            ServoMotor::setAngle(90); // Reset servo to the center position
            ultraData.size = scanSize;
            isScanning = false; // Reset scanning state
        }
    }

    return ultraData; // Return the current scan data
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
