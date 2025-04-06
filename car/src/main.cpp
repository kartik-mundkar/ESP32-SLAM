#include <Arduino.h>
#include <WiFi.h>
#include <WebSocketsServer.h>
#include "car.h"
#include "websocket_handler.h"
#include "tasks.h"
#include "config.h"

// WebSocket servers
WebSocketsServer commandWebSocket(81); // WebSocket for receiving commands
WebSocketsServer dataWebSocket(82);    // WebSocket for sending data

// Static IP configuration
IPAddress local_IP(192, 168, 137, 120);    // Replace with your desired static IP
IPAddress gateway(192, 168, 1, 1);         // Replace with your router's gateway
IPAddress subnet(255, 255, 255, 0);        // Subnet mask
IPAddress primaryDNS(8, 8, 8, 8);          // Optional: Primary DNS server
IPAddress secondaryDNS(8, 8, 4, 4);        // Optional: Secondary DNS server

// Create Car Object
int motorPins[] = {MOTOR_ENA, MOTOR_ENB, MOTOR_IN1, MOTOR_IN2, MOTOR_IN3, MOTOR_IN4};
int imuPins[] = {SDA_PIN, SCL_PIN};
int ultrasonicPins[] = {TRIGGER_PIN, ECHO_PIN};
Car car(motorPins, imuPins, ultrasonicPins, SERVO_PIN, &dataWebSocket);

void setup() {
    Serial.begin(115200);
    Serial.println("Starting ESP32 Car...");

    // Configure static IP
    if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS)) {
        Serial.println("STA Failed to configure");
    }

    // Connect to WiFi
    WiFi.setHostname("ESP32-Car");
    WiFi.begin(ssid, password); // Replace with your WiFi credentials
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.println("Connecting to WiFi...");
    }
    Serial.println("Connected to WiFi");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());

    // Initialize WebSocket servers
    commandWebSocket.begin();
    commandWebSocket.onEvent(handleCommandWebSocketEvent);

    dataWebSocket.begin();
    dataWebSocket.onEvent(handleDataWebSocketEvent);

    // Initialize car components
    car.initComponents();
    Serial.println("Car components initialized.");

    // Initialize FreeRTOS tasks
    initTasks(&car, &commandWebSocket, &dataWebSocket);
}

void loop() {
    // Process WebSocket events
    commandWebSocket.loop();
    dataWebSocket.loop();
}


