#include "../include/nonBlocking_delay.h"

// Constructor to set the delay interval
NonBlockingDelay::NonBlockingDelay(unsigned long interval) {
    previousMillis = millis();
    delayInterval = interval;
}

// Check if the delay time has elapsed
bool NonBlockingDelay::isElapsed() {
    if (millis() - previousMillis >= delayInterval) {
        previousMillis = millis();  // Reset the timer after each interval
        return true;
    }
    return false;
}

// Manually reset the timer (useful for custom timing logic)
void NonBlockingDelay::reset() {
    previousMillis = millis();
}
