/*
   Line sensor on A1,A2,A3,A4,A5
   A1-left & A5 - right
*/

#include <Arduino.h>
#include <L298N.h>

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

#define BRAKE_DELAY 30
#define MAX_SPEED 80
#define MIN_SPEED 0

// Enter Line Details
bool isBlackLine = 1;            // keep 1 in case of black line. In case of white line change this to 0
unsigned int lineThickness = 25; // Enter line thickness in mm. Works best for thickness between 10 & 35
bool brakeEnabled = 0;

#define AIN1 6
#define BIN1 8
#define AIN2 5
#define BIN2 7
#define PWMA 3
#define PWMB 9

L298N motor1(PWMA, AIN1, AIN2);
L298N motor2(PWMB, BIN1, BIN2);

int P, D, I, previousError, PIDvalue, error;
int lsp, rsp;
int lfSpeed = 60;
int currentSpeed = 60;

float Kp = 0.12;
float Kd = 1.5;
float Ki = 0;

int onLine = 1;
int minValues[7], maxValues[7], threshold[7], sensorValue[7], sensorArray[7];
bool brakeFlag = 0;

void setup()
{
    sbi(ADCSRA, ADPS2);
    cbi(ADCSRA, ADPS1);
    cbi(ADCSRA, ADPS0);

    Serial.begin(9600);

    // Set up motor control pins
    pinMode(PWMA, OUTPUT);
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(PWMB, OUTPUT);
    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);

    // Set up sensor pins
    pinMode(A1, INPUT); // Extreme left sensor
    pinMode(A2, INPUT); // Left sensor
    pinMode(A3, INPUT); // Middle sensor
    pinMode(A4, INPUT); // Right sensor
    pinMode(A5, INPUT); // Extreme right sensor

    pinMode(11, INPUT_PULLUP);
    pinMode(12, INPUT_PULLUP);
    pinMode(13, OUTPUT);
    lineThickness = constrain(lineThickness, 10, 35);
}

void linefollow()
{

    error = (3 * sensorValue[1] + sensorValue[2] - sensorValue[4] - 3 * sensorValue[5]);

    if (lineThickness > 22)
    {
        error = error * -1;
    }
    if (isBlackLine)
    {
        error = error * -1;
    }

    P = error;
    I = I + error;
    D = error - previousError;

    PIDvalue = (Kp * P) + (Ki * I) + (Kd * D);
    previousError = error;

    lsp = constrain(currentSpeed - PIDvalue, MIN_SPEED, MAX_SPEED);
    rsp = constrain(currentSpeed + PIDvalue, MIN_SPEED, MAX_SPEED);
    motor1.setSpeed(lsp);
    motor1.forward();
    motor2.setSpeed(rsp);
    motor2.forward();
}

void calibrate()
{
    for (int i = 1; i < 6; i++)
    {
        minValues[i] = analogRead(i);
        maxValues[i] = analogRead(i);
    }

    for (int i = 0; i < 10000; i++)
    {
        motor1.setSpeed(70);
        motor1.forward();
        motor2.setSpeed(70);
        motor2.backward();

        int analogPins[] = {A1, A2, A3, A4, A5};
        int numPins = sizeof(analogPins) / sizeof(analogPins[0]);

        for (int i = 0; i < numPins; i++)
        {
            int readValue = analogRead(analogPins[i]);

            if (readValue < minValues[i + 1])
            {
                minValues[i + 1] = readValue;
            }

            if (readValue > maxValues[i + 1])
            {
                maxValues[i + 1] = readValue;
            }
        }
    }

    for (int i = 1; i < 6; i++)
    {
        threshold[i] = (minValues[i] + maxValues[i]) / 2;
        Serial.print(threshold[i]);
        Serial.print(" ");
    }
    Serial.println();
    motors.stop();
}

void readLine()
{
    onLine = 0;
    int analogPins[] = {A1, A2, A3, A4, A5};
    int numPins = sizeof(analogPins) / sizeof(analogPins[0]);

    for (int i = 0; i < numPins; i++)
    {
        sensorValue[i + 1] = map(analogRead(analogPins[i]), minValues[i + 1], maxValues[i + 1], 0, 1000);
        sensorValue[i + 1] = constrain(sensorValue[i + 1], 0, 1000);

        if ((isBlackLine == 1 && sensorValue[i + 1] > 700) || (isBlackLine == 0 && sensorValue[i + 1] < 700))
        {
            onLine = 1;
        }
    }
}

void loop()
{
    calibrate();
    motor1.stop();
    motor2.stop();
    delay(1000);
    while (1)
    {
        readLine();
        if (currentSpeed < lfSpeed)
            currentSpeed++;
        if (onLine == 1)
        { // PID LINE FOLLOW
            linefollow();
            digitalWrite(13, HIGH);
            brakeFlag = 0;
        }
        else
        {
            digitalWrite(13, LOW);
            if (error > 0)
            {
                if (brakeEnabled == 1 && brakeFlag == 0)
                {
                    motor1.stop();
                    motor2.stop();
                    delay(BRAKE_DELAY);
                }
                motor1.setSpeed(150);
                motor1.backward();
                motor2.setSpeed(150);
                motor2.forward();
                brakeFlag = 1;
            }
            else
            {
                if (brakeEnabled == 1 && brakeFlag == 0)
                {
                    motor1.stop();
                    motor2.stop();
                    delay(30);
                }
                motor1.setSpeed(150);
                motor1.forward();
                motor2.setSpeed(150);
                motor2.backward();
                brakeFlag = 1;
            }
        }
    }
}