#include "../include/car.h"

Car::Car(int motorPins[], int imuPins[], int ultrasonicPins[], int servoPin)
    // Initialize the base classes with the provided pins
    : Motor(motorPins[0], motorPins[1], motorPins[2], motorPins[3], motorPins[4], motorPins[5]),
      IMUSensor(imuPins),
      UltrasonicSensor(ultrasonicPins[0], ultrasonicPins[1],servoPin) // Pass servo if needed
 {
    this->speed = 175; // Default speed
}

Car::~Car() {}

void Car::initComponents() {
    Motor::stopMotor();
    UltrasonicSensor::begin();
    IMUSensor::begin();
}

void Car::drive(char command) {
    UltraData data ;
    switch (command) {
    case 'F':
        Motor::moveForward(speed);
        isObstacleDetected();
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
        // Perform a scan and get the data
        data = UltrasonicSensor::performScan();
        // String json = UltrasonicSensor::getJson(data.angles, data.distances, MAX_SCANS);
        // Serial.println(json); // Print the JSON data to the Serial Monitor
        break;
        
    default:
        break;
    }
}

bool Car::
isObstacleDetected() {
    float currentDistance = UltrasonicSensor::getDistance();
    if (currentDistance < MIN_DISTANCE) {
        Motor::stopMotor();
        return true;
    }
    return false;
}

UltraData Car::ultraScan() {
    return UltrasonicSensor::performScan();
}

String Car::getCarData(float dt) {
    String imuJson = IMUSensor::getIMUJson(IMUSensor::readSensor(dt));
    return "{\"mode\":\"moving\",\"timestamp\":" + String(millis()) + ",\"car\":{\"imuData\":" + imuJson + "}}";
}