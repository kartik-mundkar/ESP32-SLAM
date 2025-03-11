#include "../include/bluetooth.h"

BluetoothSerial bt;  // Create only one instance globally

void setupBluetooth() {
    if (!bt.begin("ESP32-SLAM")) {  // Set Bluetooth device name
        Serial.println("Bluetooth failed to start");
        return;
    }
    Serial.println("Bluetooth Started: ESP32-SLAM");
}

void sendBluetoothData(String data) {
    bt.println(data);  // Send data over Bluetooth
}

String receiveBluetoothData() {
    if (bt.available()) {  // Check if data is available
        String receivedData = "";
        while (bt.available()) {
            char c = bt.read();
            receivedData += c;
        }
        Serial.println("Received via Bluetooth: " + receivedData);
        return receivedData;
    }
    return "";
}
