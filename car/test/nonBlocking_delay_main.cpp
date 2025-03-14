#include <Arduino.h>
#include "../include/nonBlocking_delay.h"

#define LED_PIN 2    // Example LED pin for blinking
#define SENSOR_PIN 27  // Example sensor pin

NonBlockingDelay ledDelay(1000);      // 1-second LED blink delay
NonBlockingDelay sensorDelay(500);    // 500ms sensor check delay

void setup() {
    pinMode(LED_PIN, OUTPUT);
    pinMode(SENSOR_PIN, INPUT);
    Serial.begin(115200);
}

void loop() {
    // Non-blocking LED Blink Logic
    if (ledDelay.isElapsed()) {
        static bool ledState = LOW;
        ledState = !ledState;
        digitalWrite(LED_PIN, ledState);
        Serial.println("LED Toggled!");
    }

    // Non-blocking Sensor Reading Logic
    if (sensorDelay.isElapsed()) {
        int sensorValue = digitalRead(SENSOR_PIN);
        Serial.print("Sensor Value: ");
        Serial.println(sensorValue);
    }

    // Additional tasks can run here without delays interfering
}
