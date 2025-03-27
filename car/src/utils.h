#pragma once // Include guard to prevent multiple inclusions of this header file"
// Purpose: Utility header file for the ESP32-Car project.
// dont forget to include the libraries in the platformio.ini file
// [env:esp32dev]
// platform = espressif32
// board = esp32dev
// framework = arduino

// don't mess with the order of the includes
#include <Arduino.h>            // Include Arduino framework
#include "config.h"              // Include Configuration header file
#include "../include/bluetooth.h"           // Include Bluetooth header file
#include "../include/json_utility.h"        // Include JSON Utility header file
#include "../include/imu.h"                 // Include IMU header file
#include "../include/motor.h"               // Include Motor Control header file
#include "../include/power_management.h"    // Include Power Management header file
#include "../include/ultrasonic_sensor.h"   // Include Ultrasonic Sensor header file
#include "../include/servo.h"               // Include Servo header file
#include "../include/websocket_imu.h"       // Include Websocket IMU header file
#include "car.h"                 // Include Car header file

