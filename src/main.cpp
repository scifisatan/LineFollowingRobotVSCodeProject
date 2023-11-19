// Required Libraries
#include <Arduino.h>
#include <L298NX2.h>

// Define motor control pins
// Left motor
const unsigned int EN_A = 3;
const unsigned int IN1_A = 5;
const unsigned int IN2_A = 6;

// Right motor
const unsigned int IN1_B = 7;
const unsigned int IN2_B = 8;
const unsigned int EN_B = 9;

// Global constants for sensor values
const unsigned int black = 0;
const unsigned int white = 1;

// Global constant for turning speed
const unsigned int turning_speed = 80;

// Initialize both motors
L298NX2 motors(EN_A, IN1_A, IN2_A, EN_B, IN1_B, IN2_B);

void setup()
{
    // Set up serial communication
    Serial.begin(9600);

    // Wait for Serial Monitor to be opened
    while (!Serial)
    {
        // Do nothing
    }

    // Set up motor control pins
    pinMode(EN_A, OUTPUT);
    pinMode(IN1_A, OUTPUT);
    pinMode(IN2_A, OUTPUT);
    pinMode(EN_B, OUTPUT);
    pinMode(IN1_B, OUTPUT);
    pinMode(IN2_B, OUTPUT);

    // Set up sensor pins
    pinMode(A1, INPUT); // Extreme left sensor
    pinMode(A2, INPUT); // Left sensor
    pinMode(A3, INPUT); // Middle sensor
    pinMode(A4, INPUT); // Right sensor
    pinMode(A5, INPUT); // Extreme right sensor

    // Set initial motor speeds
    motors.setSpeedA(80);
    motors.setSpeedB(80);
}

// Function to check if a sensor is on black
bool isBlack(int sensor)
{
    return (sensor == black);
}

// Function to check if a sensor is on white
bool isWhite(int sensor)
{
    return (sensor == white);
}

// Function to turn left
void turnLeft()
{
    motors.setSpeedA(turning_speed / 2); // Reduce the speed of motor A (left motor)
    motors.setSpeedB(turning_speed);     // Keep the speed of motor B (right motor)
    motors.backwardA();                  // Run motor A (left motor) backward
    motors.forwardB();                   // Run motor B (right motor) forward
}

// Function to turn right
void turnRight()
{
    motors.setSpeedA(turning_speed);     // Keep the speed of motor A (left motor)
    motors.setSpeedB(turning_speed / 2); // Reduce the speed of motor B (right motor)
    motors.forwardA();                   // Run motor A (left motor) forward
    motors.backwardB();                  // Run motor B (right motor) backward
}

// Function to turn in a circle
void turnCircle()
{
    motors.setSpeedA(turning_speed); // Keep the speed of motor A (left motor)
    motors.setSpeedB(turning_speed); // Keep the speed of motor B (right motor)
    motors.forwardA();               // Run motor A (left motor) forward
    motors.backwardB();              // Run motor B (right motor) backward
}

void loop()
{
    // Read sensor values
    int sensor1 = digitalRead(A1); // Extreme left sensor
    int sensor2 = digitalRead(A2); // Left sensor
    int sensor3 = digitalRead(A3); // Middle sensor
    int sensor4 = digitalRead(A4); // Right sensor
    int sensor5 = digitalRead(A5); // Extreme right sensor

    // Line-following logic
    if (isBlack(sensor1))
    {
        // If extreme left sensor is on black, turn left
        turnLeft();
    }
    else
    {
        if (isBlack(sensor2))
        {
            // If left sensor is on black, turn left
            turnLeft();
        }
        else if (isBlack(sensor4))
        {
            // If right sensor is on black, turn right
            turnRight();
        }
        else if (isWhite(sensor1) && isBlack(sensor3))
        {
            // If extreme left and middle sensors are on white, move forward
            motors.forward();
        }
        else if (isWhite(sensor1) && isWhite(sensor2) && isWhite(sensor3) && isWhite(sensor4) && isBlack(sensor5))
        {
            // If gap detected, turn in a circle
            turnCircle();
        }
        else if (isWhite(sensor1) && isWhite(sensor3) && isBlack(sensor5))
        {
            // If extreme right sensor is on black, turn right
            turnRight();
        }
    }
}
