#ifndef ESP32_SLAM_ULTRA_H
#define ESP32_SLAM_ULTRA_H

#include <Arduino.h>
#include <servo.h>

// Define scan range and step size
const int  SCAN_START = 0; // Start angle in degrees
const int  SCAN_END  = 180; // End angle in degrees
const int STEP_SIZE  = 5; // Step size in degrees
const int SCAN_DELAY =  50; // Delay in milliseconds between scans
const int MAX_RANGE  = 1 ;// Maximum range in meters
const int MAX_SCANS = ((SCAN_END - SCAN_START) / STEP_SIZE + 1);
const int num_of_iterations = 3; // Number of iterations for the scan

struct UltraData{
    int angles[MAX_SCANS];
    float distances[MAX_SCANS];
    int size;
};

class UltrasonicSensor: public ServoMotor {
    private:
    int triggerPin;
    int echoPin;
    UltraData ultraData;
    int scanSize;
    bool isScanning = false;


public:
    UltrasonicSensor(int trigPin, int echoPin, int servoPin);
    void begin();
    UltraData performScan();
    UltraData performScanStep();
    float getDistance();  // Returns distance in cm
    String getJson(int angles[], float distances[], int size);
};

#endif
