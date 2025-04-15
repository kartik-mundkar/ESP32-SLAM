#include "websocket_handler.h"
#include <Arduino.h>

// External references
extern Car car;
extern QueueHandle_t commandQueue;

void handleCommandWebSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length) {
    if (type == WStype_TEXT) {
        if (payload == nullptr || length == 0) {
            Serial.println("Error: Received empty or null WebSocket payload.");
            return;
        }

        String message = String((char *)payload);
        Serial.printf("Received command: %s\n", message.c_str());

        message.trim(); // Trim whitespace from the message

        // Handle "SPEED" command
        if (message.startsWith("SPEED:")) {
            int speed = message.substring(6).toInt();
            if (speed >= 0 && speed <= 255) {
                car.setSpeed(speed); // Set the speed of the car
                Serial.printf("Speed set to: %d\n", speed);
            } else {
                Serial.println("Error: Invalid speed value received. Must be between 0 and 255.");
            }
        }
        // Handle "ANGLE" command
        else if (message.startsWith("ANGLE:")) {
            int angle = message.substring(6).toInt();
            if (angle >= 0 && angle <= 180) {
                car.setServoAngle(angle);
                Serial.printf("Servo angle set to: %d\n", angle);
            } else {
                Serial.println("Error: Invalid angle value received. Must be between 0 and 180.");
            }
        }
        // Handle "CALIBRATE" command
        else if (message.startsWith("CALIBRATE")) {
            car.stopMotor(); // Stop the car before calibration
            car.calibrateIMU(); // Calibrate the IMU
            Serial.println("IMU calibrated.");
        }
        // Handle single-character commands
        else if (message.length() == 1) {
            char command = message[0];
            if (commandQueue != nullptr) {
                if (xQueueSend(commandQueue, &command, portMAX_DELAY) != pdPASS) {
                    Serial.println("Error: Failed to send command to queue.");
                }
            } else {
                Serial.println("Error: Command queue is null.");
            }
        } else {
            Serial.printf("Error: Unrecognized command format: %s\n", message.c_str());
        }
    }
    else if (type == WStype_DISCONNECTED) {
        Serial.printf("Client #%d disconnected.\n", num);
        car.stopMotor(); // Stop the car when the client disconnects
    } else if (type == WStype_CONNECTED) {
        Serial.printf("Client #%d connected.\n", num);
        car.stopMotor(); // Stop the car when a client connects
        car.calibrateIMU(); // Calibrate the IMU when a client connects
    } else {
        Serial.println("Error: Unsupported WebSocket event type.");
        Serial.printf("Event type: %d\n", type);
    }
}

void handleDataWebSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length) {
    if (type == WStype_CONNECTED) {
        Serial.printf("Data WebSocket client #%d connected.\n", num);
    } else if (type == WStype_DISCONNECTED) {
        Serial.printf("Data WebSocket client #%d disconnected.\n", num);
    }
}
