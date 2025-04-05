#include "../include/bluetooth.h"

BluetoothHandler::BluetoothHandler(const String &deviceName)
    : deviceName(deviceName) {}

void BluetoothHandler::begin() {
    this->btSerial.begin(deviceName);
    delay(1000);

    if (!this->btSerial.connected()) {
        Serial.println("Bluetooth initialized, but no devices connected.");
    } else {
        Serial.println("Bluetooth Started Successfully: " + deviceName);
    }
}

void BluetoothHandler::sendData(const String &data) {
    if (this->btSerial.connected()) {
        btSerial.println(data);
        Serial.print("Sent via Bluetooth: ");
        Serial.println(data);
    } else {
        Serial.println("Bluetooth not connected. Data not sent.");
    }
}

bool BluetoothHandler::receiveData(String &data) {
    if (this->btSerial.available()) {
        data = this->btSerial.readString(); // Improved data handling
        data.trim();                  // Removes excess spaces/newlines
        Serial.println("Received via Bluetooth: " + data);
        return true;
    }
    return false;
}

bool BluetoothHandler::isConnected(){
    return this->btSerial.connected()? true : false ;
}