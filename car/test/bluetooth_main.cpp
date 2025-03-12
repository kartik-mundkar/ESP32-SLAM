#include <Arduino.h>
#include "../include/bluetooth.h" 

#define DEVICE_NAME "ESP32_Car_Controller" // Custom Bluetooth Device Name

BluetoothHandler btHandler(DEVICE_NAME);

void setup() {
    Serial.begin(115200);    // Start serial communication for debugging
    btHandler.begin();       // Initialize Bluetooth
    Serial.println("System Ready...");
}

void loop() {
    static unsigned long previousMillis = 0;
    const long interval = 5000; // 5 seconds interval for sample data

    // Sample command data to send (simulating car movement)
    String sampleCommands[] = {"FORWARD", "BACKWARD", "LEFT", "RIGHT", "STOP"};
    static int commandIndex = 0;

    // Periodically send data
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;
        btHandler.sendData(sampleCommands[commandIndex]);
        commandIndex = (commandIndex + 1) % 5; // Loop through sample commands
    }

    // Continuously check for incoming Bluetooth data
    String receivedData;
    if (btHandler.receiveData(receivedData)) {
        Serial.print("Command Received: ");
        Serial.println(receivedData);

        // 🚗 Handle received commands
        if (receivedData == "FORWARD") {
            Serial.println("🚗 Moving Forward");
        } else if (receivedData == "BACKWARD") {
            Serial.println("🚗 Moving Backward");
        } else if (receivedData == "LEFT") {
            Serial.println("↩️ Turning Left");
        } else if (receivedData == "RIGHT") {
            Serial.println("↪️ Turning Right");
        } else if (receivedData == "STOP") {
            Serial.println("⛔ Car Stopped");
        } else {
            Serial.println("❓ Unknown Command Received");
        }
    }
}
