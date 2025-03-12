#include "../include/bluetooth.h"

BluetoothHandler::BluetoothHandler(const String &deviceName)
    : deviceName(deviceName) {}

void BluetoothHandler::begin() {
    btSerial.begin(deviceName);
    delay(1000);

    if (!btSerial.connected()) {
        Serial.println("Bluetooth initialized, but no devices connected.");
    } else {
        Serial.println("Bluetooth Started Successfully: " + deviceName);
    }
}

void BluetoothHandler::sendData(const String &data) {
    if (btSerial.connected()) {
        btSerial.println(data);
        Serial.print("Sent via Bluetooth: ");
        Serial.println(data);
    } else {
        Serial.println("Bluetooth not connected. Data not sent.");
    }
}

bool BluetoothHandler::receiveData(String &data) {
    if (btSerial.available()) {
        data = btSerial.readString(); // Improved data handling
        data.trim();                  // Removes excess spaces/newlines
        Serial.println("Received via Bluetooth: " + data);
        return true;
    }
    return false;
}
