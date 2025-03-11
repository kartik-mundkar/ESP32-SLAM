#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#include "Arduino.h"
#include "BluetoothSerial.h"

class BluetoothHandler {
public:
    BluetoothHandler(const String &deviceName);
    void begin();
    void sendData(const String &data);
    bool receiveData(String &data);

private:
    BluetoothSerial btSerial;
};

#endif // BLUETOOTH_H
