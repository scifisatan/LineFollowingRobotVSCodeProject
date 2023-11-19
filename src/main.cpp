#include <Arduino.h>
#include <L298NX2.h>

// left motor
const unsigned int EN_A = 3;
const unsigned int IN1_A = 5;
const unsigned int IN2_A = 6;

// right motor
const unsigned int IN1_B = 7;
const unsigned int IN2_B = 8;
const unsigned int EN_B = 9;

// global constants
const unsigned int turning_speed = 80;
const unsigned int black, b = 0;
const unsigned int white, w = 1;

// Initialize both motors
L298NX2 motors(EN_A, IN1_A, IN2_A, EN_B, IN1_B, IN2_B);

void setup()
{
    // put your setup code here, to run once:

    Serial.begin(9600);

    // Wait for Serial Monitor to be opened
    while (!Serial)
    {
        // do nothing
    }

    pinMode(EN_A, OUTPUT);
    pinMode(IN1_A, OUTPUT);
    pinMode(IN2_A, OUTPUT);
    pinMode(EN_B, OUTPUT);
    pinMode(IN1_B, OUTPUT);
    pinMode(IN2_B, OUTPUT);

    pinMode(A1, INPUT); // extreme left sensor
    pinMode(A2, INPUT); // left
    pinMode(A3, INPUT); // middle sensor
    pinMode(A4, INPUT); // right
    pinMode(A5, INPUT); // extreme right sensor

    motors.setSpeedA(80);
    motors.setSpeedB(80);
}

bool isOnBlack(int sensor)
{
    if (sensor == b)
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool isOnWhite(int sensor)
{
    if (sensor == w)
    {
        return true;
    }
    else
    {
        return false;
    }
}

void turnLeft()
{
    motors.setSpeedA(turning_speed / 2); // Reduce the speed of motor A
    motors.setSpeedB(turning_speed);     // Keep the speed of motor B
    motors.backwardA();                  // Run motor A forward
    motors.forwardB();                   // Run motor B forward
}

void turnRight()
{
    motors.setSpeedA(turning_speed);     // Keep the speed of motor A
    motors.setSpeedB(turning_speed / 2); // Reduce the speed of motor B
    motors.forwardA();                   // Run motor A forward
    motors.backwardB();                  // Run motor B forward
}

void turnCircle()
{
    motors.setSpeedA(turning_speed); // Keep the speed of motor A
    motors.setSpeedB(turning_speed); // Reduce the speed of motor B
    motors.forwardA();               // Run motor A forward
    motors.backwardB();              // Run motor B forward
}

void loop()
{
    // put your main code here, to run repeatedly:
    int sensor1 = digitalRead(A1); // sensor1  Extreme Left
    int sensor2 = digitalRead(A2); // sensor2
    int sensor3 = digitalRead(A3); // sensor3  middle
    int sensor4 = digitalRead(A4); // sensor4
    int sensor5 = digitalRead(A5); // sensor5   Extreme right

    if (isOnBlack(sensor1))
    {
        turnLeft();
    }
    else
    {
        if (isOnBlack(sensor2))
        {
            turnLeft();
        }
        else if (isOnBlack(sensor4))
        {
            turnRight();
        }
        else if (isOnWhite(sensor1) && isOnBlack(sensor3))
        {
            motors.forward();
        }
        else if (isOnWhite(sensor1) && isOnWhite(sensor2) && isOnWhite(sensor3) && isOnWhite(sensor4) && isOnBlack(sensor5))
        {
            // Gap detected, turn left or adjust as needed
            turnCircle();
        }
        else if (isOnWhite(sensor1) && isOnWhite(sensor3) && isOnBlack(sensor5))
        {
            turnRight();
        }
    }
}