// Purpose: Configuration file for the ESP32-Car project.
#pragma once // Include guard to prevent multiple inclusions of this header file

#define BAUD_RATE 115200 // Serial Monitor Baud Rate

// Motor driver pin configuration 
#define MOTOR_ENA  5  // PWM for Left Motor
#define MOTOR_ENB  2  // PWM for Right Motor
#define MOTOR_IN1  18  // Left Motor Forward/Reverse
#define MOTOR_IN2  19 
#define MOTOR_IN3  21 // Right Motor Forward/Reverse
#define MOTOR_IN4  22

//Ultrasonic sensor pin configuration
#define TRIGGER_PIN 26
#define ECHO_PIN 27


//Servo pin configuration
#define SERVO_PIN 13

//imu pin configuration -> default SDA (21) and SCL (22) pins 
#define SDA_PIN 32
#define SCL_PIN 33 
#define I2C_CLOCK_SPEED 400000 

//bluetooth configuration
#define BLUETOOTH_NAME "ESP32-Car"
#define BLUETOOTH_PIN "1234"

// //wifi-connection configuartion
const char* ssid = "KARTIK 2824";
const char* password = "84Q38#3u";
