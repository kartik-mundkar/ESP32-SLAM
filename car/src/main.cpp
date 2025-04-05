#include <Arduino.h>
#include <WiFi.h>
#include <WebSocketsServer.h>
#include "car.h"
#include "config.h"

WebSocketsServer webSocket(81); // WebSocket server on port 81
// Define network parameters
IPAddress local_IP(192, 168, 137 ,120); // Your desired static IP
IPAddress gateway(192, 168, 1, 1);  // Your router's IP (gateway)
IPAddress subnet(255, 255, 255, 0); // Subnet mask
IPAddress primaryDNS(8, 8, 8, 8); // Optional: Google DNS
IPAddress secondaryDNS(8, 8, 4, 4); // Optional: Google DNS

// Create Car Object
int motorPins[] = {MOTOR_ENA, MOTOR_ENB, MOTOR_IN1, MOTOR_IN2, MOTOR_IN3, MOTOR_IN4};
int imuPins[] = {SDA_PIN, SCL_PIN};
int ultrasonicPins[] = {TRIGGER_PIN, ECHO_PIN};
int servoPin = SERVO_PIN; // Servo pin for ultrasonic sensor
Car car(motorPins, imuPins, ultrasonicPins, servoPin); // Create Car object

unsigned long lastTime = 0;
static unsigned long lastWebSocketTime = 0;

// WebSocket event handler
void handleWebSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length) {
    switch (type) {
    case WStype_CONNECTED:
        Serial.printf("[WebSocket] Client %u connected.\n", num);
        break;

    case WStype_DISCONNECTED:
        Serial.printf("[WebSocket] Client %u disconnected.\n", num);
        break;

    case WStype_TEXT:
        Serial.printf("[WebSocket] Received command: %s\n", payload);
        car.drive((char)payload[0]); // Use the first character as the command
        break;

    default:
        break;
    }
}

void setup() {
    Serial.begin(115200);
    Serial.println("Starting ESP32 Car...");

    // Configures static IP address
    if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS)) {
        Serial.println("STA Failed to configure");
    }

    // Connect to WiFi
    WiFi.begin(ssid, password); // Replace with your WiFi credentials
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.println("Connecting to WiFi...");
    }
    Serial.println("Connected to WiFi");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());

    car.initComponents(); // Initialize all components
    Serial.println("Car components initialized.");
    // Start WebSocket server
    webSocket.begin();
    webSocket.onEvent(handleWebSocketEvent);
    Serial.println("WebSocket server started on port 81.");
}

void loop() {
    unsigned long currentTime = millis();
    float dt = (currentTime - lastTime) / 1000.0;
    lastTime = currentTime;
    // Send IMU data via WebSocket every 50 ms
    // if (millis() - lastWebSocketTime >= 50) {
    //     lastWebSocketTime = millis();
    //     String imuData = car.getCarData(dt); // Get IMU data in JSON format
    //     // Serial.println(imuData); // Print IMU data to Serial Monitor
    //     webSocket.broadcastTXT(imuData); // Broadcast IMU data to all connected clients
    // }
    String imuData = car.getCarData(dt); // Get IMU data in JSON format
    Serial.println(imuData); // Print IMU data to Serial Monitor
    webSocket.broadcastTXT(imuData); // Broadcast IMU data to all connected clients
    
    // Handle WebSocket events
    webSocket.loop();
}


