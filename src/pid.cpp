#include <Arduino.h>
#include <L298N.h>

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

#define BRAKE_DELAY 30
#define MAX_SPEED 100
#define MIN_SPEED 0

// Enter Line Details
bool isBlackLine = 1;            // keep 1 in case of black line. In case of white line change this to 0
unsigned int lineThickness = 25; // Enter line thickness in mm. Works best for thickness between 10 & 35

bool brakeEnabled = 0;

#define AIN1 5
#define BIN1 7
#define AIN2 6
#define BIN2 8
#define PWMA 3
#define PWMB 9

// these constants are used to allow you to make your motor configuration
// line up with function names like forward. Value can be 1 or -1
const int offsetA = 1;
const int offsetB = 1;

// Initializing motors. The library will allow you to initialize as many
// motors as you have memory for. If you are using functions like forward
// that take 2 motors as arguements you can either write new functions or
// call the function more than once.

L298N motor1(AIN1, AIN2, PWMA);
L298N motor2(BIN1, BIN2, PWMB);

int P, D, I, previousError, PIDvalue, error;
int lsp, rsp;
int lfSpeed = 70;
int currentSpeed = 30;

float Kp = 0.06;
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
    pinMode(11, INPUT_PULLUP);
    pinMode(12, INPUT_PULLUP);
    pinMode(13, OUTPUT);
    lineThickness = constrain(lineThickness, 10, 35);
}

void loop()
{
    while (digitalRead(11))
    {
    }
    delay(1000);
    calibrate();
    while (digitalRead(12))
    {
    }
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
    for (int i = 0; i < 7; i++)
    {
        minValues[i] = analogRead(i);
        maxValues[i] = analogRead(i);
    }

    for (int i = 0; i < 10000; i++)
    {
        motor1.setSpeed(50);
        motor1.forward();
        motor2.setSpeed(50);
        motor2.forward();

        for (int i = 0; i < 7; i++)
        {
            if (analogRead(i) < minValues[i])
            {
                minValues[i] = analogRead(i);
            }
            if (analogRead(i) > maxValues[i])
            {
                maxValues[i] = analogRead(i);
            }
        }
    }

    for (int i = 0; i < 7; i++)
    {
        threshold[i] = (minValues[i] + maxValues[i]) / 2;
        Serial.print(threshold[i]);
        Serial.print(" ");
    }
    Serial.println();

    motor1.stop();
    motor2.stop();
}

void readLine()
{
    onLine = 0;

    for (int i = 1; i < 6; i++)
    {
        sensorValue[i] = map(analogRead(i), minValues[i], maxValues[i], 0, 1000);
        sensorValue[i] = constrain(sensorValue[i], 0, 1000);
        if (isBlackLine == 1 && sensorValue[i] > 700)
            onLine = 1;
        if (isBlackLine == 0 && sensorValue[i] < 700)
            onLine = 1;
    }
}