#include "../include/car.h"

Car::Car(int motorPins[], int imuPins[], int ultrasonicPins[], int servoPin, WebSocketsServer *webSocket)
    // Initialize the base classes with the provided pins
    : Motor(motorPins[0], motorPins[1], motorPins[2], motorPins[3], motorPins[4], motorPins[5]),
      IMUSensor(imuPins),
      UltrasonicSensor(ultrasonicPins[0], ultrasonicPins[1],servoPin) // Pass servo if needed
 {
    this->speed = 250; // Default speed
    this->webSocket = webSocket; // Store the WebSocket reference
}

Car::~Car() {}

void Car::initComponents() {
    Motor::stopMotor();
    UltrasonicSensor::begin();
    IMUSensor::begin();
}

void scanTask(void *parameter) {
    Car *car = (Car *)parameter; // Cast the parameter to a Car object

    // Perform the scan
    UltraData data = car->UltrasonicSensor::performScan(); // Perform the scan
    String scanJson = car->UltrasonicSensor::getJson(data.angles, data.distances, data.size); // Convert the scan data to JSON
    String jsonResponse = "{\"mode\":\"scanning\",\"timestamp\":" + String(millis()) + ",\"scan\":" + scanJson + "}";

    // Send the scan data via WebSocket
    if (car->webSocket != nullptr) {
        car->webSocket->broadcastTXT(jsonResponse);
    } else {
        Serial.println("Error: WebSocket reference is null.");
    }

    // Print the scan data to the Serial Monitor
    Serial.println("Scan data sent: " + jsonResponse);
    Serial.println("Scan complete.");

    // Delete the task after completion
    vTaskDelete(NULL);
}

void Car::drive(char command) {

    switch (command) {
    case 'F':
        Motor::moveForward(speed);
        // isObstacleDetected();
        break;
    case 'B':
        Motor::moveBackward(speed);
        break;
    case 'L':
        Motor::turnLeft(speed);
        break;
    case 'R':
        Motor::turnRight(speed);
        break;
    case 'S':
        Motor::stopMotor();
        break;
    case 'C':
        xTaskCreate(scanTask, "ScanTask", 4096, this, 1, NULL); // Create a new task for scanning        
        break;  
    default:
        break;
    }
}

bool Car::isObstacleDetected() {
    float currentDistance = UltrasonicSensor::getDistance();
    if (currentDistance < MIN_DISTANCE) {
        Motor::stopMotor();
        return true;
    }
    return false;
}

String Car::getCarData(float dt) {
    String imuJson = IMUSensor::getIMUJson(IMUSensor::readSensor(dt));
    return "{\"mode\":\"moving\",\"timestamp\":" + String(millis()) + ",\"car\":{ \"imu\": "+ imuJson + ",\"ultra\":" + UltrasonicSensor::getDistance() + "}}";
}

// void Car::setSpeed(int speed) {
//     if (speed >= 0 && speed <= 255) { // Ensure speed is within valid range
//         this->speed = speed;
//         Motor::setSpeed(speed); // Set motor speed
//     } else {
//         Serial.println("Invalid speed value. Speed must be between 0 and 255.");
//     }
// }

void Car::setServoAngle(int angle) {
    UltrasonicSensor::setAngle(angle); // Use the UltrasonicSensor's setAngle method
    Serial.printf("Servo angle updated to: %d\n", angle);
}

void Car::handleScan() {
    UltraData data = UltrasonicSensor::performScan(); // Perform the scan
    String scanJson = UltrasonicSensor::getJson(data.angles, data.distances, data.size); // Convert the scan data to JSON
    String jsonResponse = "{\"mode\":\"scanning\",\"timestamp\":" + String(millis()) + ",\"scan\":" + scanJson + "}";
    webSocket->broadcastTXT(jsonResponse); // Broadcast the scan data
    Serial.println("Scan data sent: " + jsonResponse); // Print the scan data to the Serial Monitor
    Serial.println("Scan complete.");
}