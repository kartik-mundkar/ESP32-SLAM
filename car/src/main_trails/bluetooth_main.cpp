#include <Arduino.h>
#include "bluetooth.h"

void setup() {
    Serial.begin(115200);  // Start Serial Monitor
    setupBluetooth();      // Initialize Bluetooth
}

void loop() {
    String receivedData = receiveBluetoothData();  // Check for incoming data
    if (receivedData.length() > 0) {  
        Serial.println("Processing Received Data: " + receivedData);
        sendBluetoothData("ESP32 Received: " + receivedData);  // Send response
    }
    delay(100);  // Short delay to avoid unnecessary looping
}
