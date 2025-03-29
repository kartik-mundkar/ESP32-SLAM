#ifndef ESP32_SLAM_ULTRASONIC_SENSOR_H
#define ESP32_SLAM_ULTRASONIC_SENSOR_H

#include <Arduino.h>

class UltrasonicSensor {
private:
    int triggerPin;
    int echoPin;

public:
    UltrasonicSensor(int trigPin, int echoPin);
    void begin();
    float getDistance();  // Returns distance in cm
    String get180Scan();  // Returns distance in cm for 180 degree scan
};

#endif
