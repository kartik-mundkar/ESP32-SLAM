#include <Arduino.h>
#include "../include/imu.h" 

IMUSensor imu;  // Create an instance of IMUSensor
MPU9250_asukiaaa  mpu;

void setup() {
    Serial.begin(115200);
    imu.begin();
        
    
}

void loop() {
    float roll, pitch;
    float ax, ay, az, gx, gy, gz, mx, my, mz;

    // Get Orientation Data
    imu.estimateOrientation(roll, pitch);
    Serial.print("Roll: "); Serial.print(roll);
    Serial.print(" | Pitch: "); Serial.println(pitch);

    // Get Full IMU Data
    imu.readIMUData(ax, ay, az, gx, gy, gz, mx, my, mz);
    Serial.print("Accel X: "); Serial.print(ax);
    Serial.print(" | Accel Y: "); Serial.print(ay);
    Serial.print(" | Accel Z: "); Serial.println(az);

    Serial.print("Gyro X: "); Serial.print(gx);
    Serial.print(" | Gyro Y: "); Serial.print(gy);
    Serial.print(" | Gyro Z: "); Serial.println(gz);

    Serial.print("Mag X: "); Serial.print(mx);
    Serial.print(" | Mag Y: "); Serial.print(my);
    Serial.print(" | Mag Z: "); Serial.println(mz);

    Serial.println("--------------------------------");
    delay(500);  // Delay for readability
}
