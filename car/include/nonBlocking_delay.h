#ifndef NONBLOCKING_DELAY_H
#define NONBLOCKING_DELAY_H

#include <Arduino.h>

class NonBlockingDelay {
private:
    unsigned long previousMillis; // Stores the last recorded time
    unsigned long delayInterval;  // Delay interval in milliseconds

public:
    NonBlockingDelay(unsigned long interval);  // Constructor
    bool isElapsed();                         // Check if delay period has elapsed
    void reset();                             // Reset the timer
};

#endif // NONBLOCKING_DELAY_H
