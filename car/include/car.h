#ifndef ESP32_SLAM_CAR_H
#define ESP32_SLAM_CAR_H

#include <Arduino.h>
#include "motor.h"
#include "servo.h"
#include "imu.h"
#include "ultrasonic_sensor.h"
#include <WebSocketsServer.h> // Include the WebSocket library

const int MIN_DISTANCE = 20;

class Car : public Motor, public IMUSensor, public UltrasonicSensor {
private:
int speed;

public:
    WebSocketsServer *webSocket; // Pointer to the WebSocket server
    Car(int motorPins[], int imuPins[], int ultrasonicPins[], int servoPin, WebSocketsServer *webSocket);
    ~Car();
    void initComponents();
    void drive(char command);
    bool isObstacleDetected();
    String getCarData(float dt);
    int getSpeed() const { return speed; } // Getter for speed
    void setServoAngle(int angle); // Method to set the servo angle
};

#endif // ESP32_SLAM_CAR_H
