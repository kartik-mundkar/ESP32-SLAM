#include "../include/car.h"

Car::Car(BluetoothHandler *bt, Motor *motor, ServoMotor *servo, IMUSensor *imu, UltrasonicSensor *ultrasonic_sensor)
{
    this->speed = 100;
    this->motor = motor;
    this->servo = servo;
    this->imu = imu;
    this->ultrasonic_sensor = ultrasonic_sensor;
    this->bt = bt;
    Serial.println("Starting ESP32 Car...");
}
Car::~Car()
{
}

void Car::initComponents()
{
    this->bt->begin();
    this->motor->stopMotor();
    this->servo->begin();
    this->imu->begin();
    this->ultrasonic_sensor->begin();
    Serial.println("All components initialized successfully!");
}

bool Car::isObstacleDetected()
{
    float currentDistance = this->ultrasonic_sensor->getDistance();
    if (currentDistance < MIN_DISTANCE)
    {
        this->motor->stopMotor();
        Serial.println("Obstacle Detected");
        this->bt->sendData("Obstacle Detected");
        return true;
    }
    return false;
}

void Car::drive(char command)
{
    if (!this->bt->isConnected())
    {
        return;
    }
    switch (command)
    {
    case 'F':
        this->motor->moveForward(speed);
        servo->setAngle(90);
        if (isObstacleDetected())
        {
            this->motor->stopMotor();
            Serial.println("Obstacle Detected");
            this->bt->sendData("Obstacle Detected");
            this->ultrasonicData = get180Scan();
            return;
        }
        Serial.println("Moving Forward");
        this->bt->sendData("Moving Forward");
        this->bt->sendData("Speed: " + String(speed));
        this->bt->sendData("Distance: " + String(this->ultrasonic_sensor->getDistance()));
        break;

    case 'B':
        this->motor->moveBackward(speed);
        Serial.println("Moving Backward");
        this->bt->sendData("Moving Backward");
        break;

    case 'L':
        this->motor->turnLeft(speed);
        Serial.println("Turning Left");
        this->bt->sendData("Turning Left");
        break;

    case 'R':
        this->motor->turnRight(speed);
        Serial.println("Turning Right");
        this->bt->sendData("Turning Right");
        break;

    case 'S':
        this->motor->stopMotor();
        Serial.println("Stopping Motors");
        this->bt->sendData("Stopping Motors");
        break;

    case 'C':
        this->motor->stopMotor();
        Serial.println("Ultrasonic Scan Triggered!");
        this->bt->sendData("Ultrasonic Scan Triggered!");
        this->scanData = get180Scan();
        this->bt->sendData(scanData);
        break;
    default:
        break;
    }
}

String Car::getIMU_JSONData()
{
    float ax, ay, az, gx, gy, gz, mx, my, mz, roll, pitch, yaw;
    this->imu->readIMUData(ax, ay, az, gx, gy, gz, mx, my, mz);
    this->imu->estimateOrientation(roll, pitch, yaw);
    return JsonUtility::createJson({{"ax", ax}, {"ay", ay}, {"az", az}, {"gx", gx}, {"gy", gy}, {"gz", gz}, {"mx", mx}, {"my", my}, {"mz", mz}, {"roll", roll}, {"pitch", pitch}, {"yaw", yaw}});
}

String Car::getDistance()
{
    return JsonUtility::createJson({{"distance", this->ultrasonic_sensor->getDistance()}});
}

String Car::get180Scan()
{
    return this->ultrasonic_sensor->get180Scan();
}