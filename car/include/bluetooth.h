#ifndef ESP32_SLAM_BLUETOOTH_H
#define ESP32_SLAM_BLUETOOTH_H

#include "Arduino.h"
#include "BluetoothSerial.h"

class BluetoothHandler {
public:
    BluetoothHandler(const String &deviceName);
    void begin();
    void sendData(const String &data);
    bool receiveData(String &data);
    bool isConnected();

private:
    BluetoothSerial btSerial;
    String deviceName;
};

#endif // BLUETOOTH_H
