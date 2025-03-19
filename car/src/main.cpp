#include "config.h"
#include "utils.h"
#include <Arduino.h>
#define bt_device_name "ESP32_ROBOT"
// Create Object Instances
BluetoothHandler bt(bt_device_name);  // Bluetooth device name
IMUSensor imu;
UltrasonicSensor ultrasonic(TRIGGER_PIN, ECHO_PIN);
ServoMotor servo(SERVO_PIN);
NonBlockingDelay normal_delay(300);

TaskHandle_t UltrasonicTaskHandle = NULL;

// Function Declarations
void UltrasonicScanTask(void *pvParameters);
void startUltrasonicScan();

// Global Variables
bool scanning = false;
int scanAngle = 0;



// void scanUltrasonic() {
//     Serial.println("Scanning with Ultrasonic Sensor...");

//     for (int angle = 0; angle <= 180; angle += 10) {
//         servo.setAngle(angle);  // Move servo
//         normal_delay.reset();
//         normal_delay.isElapsed();  // Wait for stabilization
        
//         float distance = ultrasonic.getDistance();  // Get distance
        
//         // Format and send data
//         String sensorData = "Angle: " + String(angle) +
//                             " | Distance: " + String(distance) + " cm";
//         bt.sendData(sensorData);
//         Serial.println(sensorData);
//     }

//     servo.setAngle(90);

// }

// Function to Start Ultrasonic Scan Task
void startUltrasonicScan() {
    if (!scanning) {
        scanning = true;  // Set scanning flag
        scanAngle = 0;    // Reset scan angle
    }
}

// FreeRTOS Task for Ultrasonic Scanning (Runs Independently)
void UltrasonicScanTask(void *pvParameters) {
    while (1) {
        if (scanning) {
            if (scanAngle <= 180) {
                servo.setAngle(scanAngle);
                vTaskDelay(pdMS_TO_TICKS(300));  // Non-blocking delay

                float distance = ultrasonic.getDistance();

                // Send data over Bluetooth
                String sensorData = "Angle: " + String(scanAngle) +
                                    " | Distance: " + String(distance) + " cm";
                bt.sendData(sensorData);
                Serial.println(sensorData);

                scanAngle += 10;
            } else {
                servo.setAngle(90);  // Reset servo to center position
                scanning = false;    // Stop scanning
                Serial.println("🔄 Servo Reset to 90° - Scanning Complete");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(50));  // Small delay to avoid CPU overload
    }
}

void setup() {
    Serial.begin(115200);

    // Initialize Components
    bt.begin();
    imu.begin();
    ultrasonic.begin();
    servo.begin();
    setupMotors();  // Initialize motor driver

    xTaskCreatePinnedToCore(
        UltrasonicScanTask,  // Task function
        "UltrasonicScan",    // Task name
        4096,                // Stack size
        NULL,                // Task parameters
        1,                   // Priority
        &UltrasonicTaskHandle, // Task handle
        0                    // Run on Core 0
    );

    Serial.println("All components initialized successfully!");
}



void loop() {
    // Check if Bluetooth is connected
    if (bt.isConnected()) {
        Serial.println("Bluetooth Connected");

        // Read IMU Data
        float ax, ay, az, gx, gy, gz, mx, my, mz;
        imu.readIMUData(ax, ay, az, gx, gy, gz, mx, my, mz);
        float roll, pitch;
        imu.estimateOrientation(roll, pitch);

        // Send IMU Data Over Bluetooth
        String imuData = "IMU -> ax: " + String(ax) + ", ay: " + String(ay) + ", az: " + String(az) +
                         " | gx: " + String(gx) + ", gy: " + String(gy) + ", gz: " + String(gz) +
                         " | mx: " + String(mx) + ", my: " + String(my) + ", mz: " + String(mz) +
                         " | Roll: " + String(roll) + ", Pitch: " + String(pitch);
        // bt.sendData(imuData);
        Serial.println(imuData);

        //set speed
        int sf=70;
        int sb=50;
        int slr=50;
        
        String command;
        if (bt.receiveData(command)) {  // Read command from Bluetooth
            Serial.println("Command Received: " + command);

            switch (command[0]) {  // Check the first character of the command
                case 'F': 
                {
                    
                    moveForward(sf);
                    Serial.println("Moving Forward");
                    float distance_f = ultrasonic.getDistance();
                    Serial.println("Calculating forward distance..." + (int) distance_f);
                    if(distance_f<=20.0)
                    {
                        moveForward(sf/2);
                        Serial.println("speed reduced to half");
                    }
                    if(distance_f<=10.0)
                    {
                        stopCar();
                        Serial.println("stop Car");
                        break;
                    }
                }
                break;
                                        
                case 'B': 
                    moveBackward(sb);
                    Serial.println("Moving Backward");
                    break;
                case 'L': 
                    turnLeft(slr);
                    Serial.println("Turning Left");
                    break;
                case 'R': 
                    turnRight(slr);
                    Serial.println("Turning Right");
                    break;
                case 'S': 
                    stopCar();
                    Serial.println("Stopping Motors");
                    break;
                case 'C':  // Manually trigger ultrasonic scanning
                    stopCar();
                    Serial.println("Manual Ultrasonic Scan Triggered!");
                    startUltrasonicScan();
                    break;
                default:
                    Serial.println("Invalid Command");
                    break;
            }
   }
    } else {
        Serial.println("Bluetooth Not Connected...");
    }

    delay(500); 
}


