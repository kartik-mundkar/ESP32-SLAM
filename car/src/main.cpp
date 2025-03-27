#include "utils.h"
#include <Arduino.h>

// Create Object Instances
BluetoothHandler bt(BLUETOOTH_NAME);                                        // Bluetooth Handler
IMUSensor imu;                                                              // IMU sensor
UltrasonicSensor ultrasonic_sensor(TRIGGER_PIN, ECHO_PIN);                         // Ultrasonic sensor
ServoMotor servo(SERVO_PIN);                                                // Servo motor
Motor motor1(MOTOR_ENA,MOTOR_ENB,MOTOR_IN1,MOTOR_IN2,MOTOR_IN3,MOTOR_IN4);   // Motor
Car car(&bt, &motor1, &servo, &imu, &ultrasonic_sensor);                            // Car

void setup()
{
    Serial.begin(BAUD_RATE); // Initialize Serial Monitor
    Serial.println("Starting ESP32 Car...");
    car.initComponents();                                                  // Initialize all components
}

void loop()
{

    if (bt.isConnected())
    {
        String command;
        const String IMU_JSON = (const String) car.getIMU_JSONData();
        bt.sendData(IMU_JSON);
        Serial.println(IMU_JSON);
        if (bt.receiveData(command))
        {
            Serial.println("Received Command: " + command);
            car.drive(command[0]);
        }      

        delay(1000);
    }
    else
    {
        Serial.println("Bluetooth Not Connected...");
    }
}
