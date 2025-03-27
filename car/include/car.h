#ifndef ESP32_SLAM_CAR_H
#define ESP32_SLAM_CAR_H
#include <Arduino.h>
#include "motor.h"
#include "servo.h"
#include "imu.h"
#include "ultrasonic_sensor.h"
#include "bluetooth.h"
#include "json_utility.h"


const int MIN_DISTANCE = 20;

class Car
{
private:
    Motor *motor;
    ServoMotor *servo;
    IMUSensor *imu;
    UltrasonicSensor *ultrasonic_sensor;
    BluetoothHandler *bt;
    int speed;
    String scanData;
    String ultrasonicData;
    
    public:
    Car(BluetoothHandler *bt, Motor *motor, ServoMotor *servo, IMUSensor *imu, UltrasonicSensor *ultrasonic_sensor);
    ~Car();
    void initComponents();
    void drive(char command);
    bool isObstacleDetected();
    String getIMU_JSONData();
    String get180Scan();
    String getDistance();
};

#endif // CAR_H
