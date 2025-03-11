#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>

// Define motor control pins
#define ENA  5  // PWM for Left Motor
#define ENB  2  // PWM for Right Motor
#define IN1  18  // Left Motor Forward/Reverse
#define IN2  19 
#define IN3  21 // Right Motor Forward/Reverse
#define IN4  22 

// Function prototypes
void setupMotors();
void moveForward(int speed);
void moveBackward(int speed);
void turnLeft(int speed);
void turnRight(int speed);
void stopCar();

#endif