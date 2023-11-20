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

void loop()
{
    // Read sensor values
    int sensor1 = digitalRead(A1); // Extreme left sensor
    int sensor2 = digitalRead(A2); // Left sensor
    int sensor3 = digitalRead(A3); // Middle sensor
    int sensor4 = digitalRead(A4); // Right sensor
    int sensor5 = digitalRead(A5); // Extreme right sensor

    Serial.println(sensor1);
    Serial.println(sensor2);
    Serial.println(sensor3);
    Serial.println(sensor4);
    Serial.println(sensor5);
    motors.forward();
    delay(1000);
    motors.backward();
    delay(1000);
}
