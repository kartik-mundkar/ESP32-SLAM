#include "../include/motor.h"
#include <Arduino.h>

Motor::Motor(int ENA, int ENB, int IN1, int IN2, int IN3, int IN4)
{
    this->ENA = ENA;
    this->ENB = ENB;
    this->IN1 = IN1;
    this->IN2 = IN2;
    this->IN3 = IN3;
    this->IN4 = IN4;
        
    pinMode(ENA, OUTPUT);
    pinMode(ENB, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
}

// Move forward
void Motor::moveForward(int speed)
{
    analogWrite(ENA, speed);
    analogWrite(ENB, speed);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
}

// Move backward
void Motor::moveBackward(int speed)
{
    analogWrite(ENA, speed);
    analogWrite(ENB, speed);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
}

// Turn left
void Motor::turnLeft(int speed)
{
    analogWrite(ENA, speed);
    analogWrite(ENB, speed);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
}

// Turn right
void Motor::turnRight(int speed)
{
    analogWrite(ENA, speed);
    analogWrite(ENB, speed);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
}

// Stop the car
void Motor::stopMotor()
{
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
}

// Set speed for both motors
void Motor::setSpeed(int speed)
{
    analogWrite(ENA, speed);
    analogWrite(ENB, speed);
}
