#include <Arduino.h>
#include <MOTORS.h>
#include <ARRAY.h>
#include <PID.h>

// Global constants
#define BRAKE_DELAY 30
#define MAX_SPEED 90
#define MIN_SPEED 50
const unsigned int black = 0;
const unsigned int white = 1;
bool onLine = 1;
bool brakeFlag = 1;
int currentSpeed = 50;
int lfSpeed = 60;
bool brakeEnabled = 1;
int leftSpeed, rightSpeed, error, PIDvalue;
int isBlackLine = 1;

// PID constants
float Kp = 0;
float Kd = 0;
float Ki = 0;

MOTORS motors(3, 5, 6, 7, 8, 9);

ARRAY array(A0, A1, A2, A3, A4);

PID pid(Kp, Ki, Kd);

void printSensorValues(int sensor1, int sensor2, int sensor3, int sensor4, int sensor5);

void setup()
{
    Serial.begin(9600);
    int sensor1 = array.IRvalues[0]; // Extreme left sensor
    int sensor2 = array.IRvalues[1]; // Left sensor
    int sensor3 = array.IRvalues[2]; // Middle sensor
    int sensor4 = array.IRvalues[3]; // Right sensor
    int sensor5 = array.IRvalues[4]; // Extreme right sensor
}

void loop()
{
    // Read sensor values
    motors.turnAround(60);
    array.readSensorValue();
    array.calibrate();
    motors.stop();
    delay(1000);

    while (1)
    {
        onLine = array.isOnLine();
        if (currentSpeed < lfSpeed)
            currentSpeed++;
        if (onLine == 1)
        { // PID LINE FOLLOW
            error = isBlackLine ? array.calcError() : (-1 * array.calcError());
            PIDvalue = pid.calcPID(error);
            leftSpeed = currentSpeed - PIDvalue;
            rightSpeed = currentSpeed + PIDvalue;
            if ((leftSpeed > 0) && (rightSpeed > 0))
            {
                leftSpeed = constrain(leftSpeed, MIN_SPEED, MAX_SPEED);
                rightSpeed = constrain(rightSpeed, MIN_SPEED, MAX_SPEED);
            }
            else if ((leftSpeed < 0) && (rightSpeed < 0))
            {
                leftSpeed = constrain(leftSpeed, -MAX_SPEED, -MIN_SPEED);
                rightSpeed = constrain(rightSpeed, -MAX_SPEED, -MIN_SPEED);
            }
            else if ((leftSpeed < 0) && (rightSpeed > 0))
            {
                leftSpeed = constrain(leftSpeed, -MAX_SPEED, -MIN_SPEED);
                rightSpeed = constrain(rightSpeed, MIN_SPEED, MAX_SPEED);
            }
            else if ((leftSpeed > 0) && (rightSpeed < 0))
            {
                leftSpeed = constrain(leftSpeed, MIN_SPEED, MAX_SPEED);
                rightSpeed = constrain(rightSpeed, -MAX_SPEED, -MIN_SPEED);
            }
            motors.drive(leftSpeed, rightSpeed);
            brakeFlag = 0;
        }
        else
        { // PID LINE SEARCH
            if (error > 0)
            {
                if (brakeEnabled == 1 && brakeFlag == 0)
                {
                    motors.stop();
                    delay(BRAKE_DELAY);
                }
                motors.turnAround(75);
                brakeFlag = 1;
            }
            else
            {
                if (brakeEnabled == 1 && brakeFlag == 0)
                {
                    motors.stop();
                    delay(BRAKE_DELAY);
                }
                motors.turnAround(75);
                brakeFlag = 1;
            }
        }
    }
    delay(1000);
}

void printSensorValues(int sensor1, int sensor2, int sensor3, int sensor4, int sensor5)
{
    Serial.println("------------------ Sensor Values ------------------");
    Serial.print("Sensor1: ");
    Serial.println(array.IRvalues[0]);
    Serial.print("Sensor2: ");
    Serial.println(array.IRvalues[1]);
    Serial.print("Sensor3: ");
    Serial.println(array.IRvalues[2]);
    Serial.print("Sensor4: ");
    Serial.println(array.IRvalues[3]);
    Serial.print("Sensor5: ");
    Serial.println(array.IRvalues[4]);
    Serial.println("----------------------------------------------------");
}
