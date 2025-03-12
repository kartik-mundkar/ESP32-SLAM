#include "config.h"
#include "utils.h"
#include<Arduino.h>

// Create Object Instances
BluetoothHandler bt("ESP32_ROBOT");  // Bluetooth device name
IMUSensor imu;
UltrasonicSensor ultrasonic(TRIGGER_PIN, ECHO_PIN);
ServoMotor servo(SERVO_PIN);


void setup() {
    Serial.begin(115200);

    // Initialize Components
    bt.begin();
    imu.begin();
    ultrasonic.begin();
    servo.begin();
    setupMotors();  // Initialize motor driver

    Serial.println("All components initialized successfully!");
}

void loop() {
    // Check if Bluetooth is connected
    if (bt.isConnected()) {
        Serial.println("Bluetooth Connected");

        // Read sensor data
        float ax, ay, az, gx, gy, gz, mx, my, mz;
        imu.readIMUData(ax, ay, az, gx, gy, gz, mx, my, mz);
        float roll, pitch;
        imu.estimateOrientation(roll, pitch);

        //float distance = ultrasonic.getDistance();

        // Read Bluetooth command
        String command;
        if (bt.receiveData(command)) {
            Serial.println("Command Received: " + command);

            // Motor Control Commands
            if (command == "F") {
                moveForward(150);  // Move forward at speed 150
            } else if (command == "B") {
                moveBackward(150); // Move backward at speed 150
            } else if (command == "L") {
                turnLeft(150);     // Turn left at speed 150
            } else if (command == "R") {
                turnRight(150);    // Turn right at speed 150
            } else if (command == "S") {
                stopCar();         // Stop motors
            }

    
        }

        for (int angle = 0; angle <= 180; angle += 10) {
            servo.setAngle(angle); // Move servo to new position
            delay(200);            // Allow servo to stabilize

            float distance = ultrasonic.getDistance(); // Read distance sensor

            // Format and send data
            String sensorData = "Angle: " + String(angle) +
                                " | Distance: " + String(distance) + " cm" +
                                " | IMU: ax=" + String(ax) + ", ay=" + String(ay) + ", az=" + String(az) +
                                " | gx=" + String(gx) + ", gy=" + String(gy) + ", gz=" + String(gz) +
                                " | mx=" + String(mx) + ", my=" + String(my) + ", mz=" + String(mz);
            bt.sendData(sensorData);

        }

    } else {
        Serial.println(" Bluetooth Not Connected...");
    }

    delay(500); 
}


