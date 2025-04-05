#ifndef ESP32_SLAM_ULTRA_H
#define ESP32_SLAM_ULTRA_H

#include <Arduino.h>
#include <servo.h>

// Define scan range and step size
#define  SCAN_START  0
#define  SCAN_END  180
#define STEP_SIZE  10
#define MAX_SCANS  ((SCAN_END - SCAN_START) / STEP_SIZE + 1)
#define MAX_RANGE  400 //ma

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

public:
    UltrasonicSensor(int trigPin, int echoPin, int servoPin);
    void begin();
    UltraData performScan();
    float getDistance();  // Returns distance in cm
    String getJson(int angles[], float distances[], int size);
};

#endif
