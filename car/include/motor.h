#ifndef ESP32_SLAM_MOTOR_H
#define ESP32_SLAM_MOTOR_H
class Motor
{
private:
    int ENA, ENB, IN1, IN2, IN3, IN4;
public:
    Motor(int ENA, int ENB, int IN1, int IN2, int IN3, int IN4);
    void moveForward(int speed);
    void moveBackward(int speed);
    void turnLeft(int speed);
    void turnRight(int speed);
    void stopMotor();
    void setSpeed(int speed);
};
#endif // ESP32_SLAM_MOTOR_H