#ifndef ULTRASONIC_SENSOR_H
#define ULTRASONIC_SENSOR_H

#include <Arduino.h>

class UltrasonicSensor {
private:
    int triggerPin;
    int echoPin;

public:
    UltrasonicSensor(int trigPin, int echoPin);
    void begin();
    float getDistance();  // Returns distance in cm
};

#endif
