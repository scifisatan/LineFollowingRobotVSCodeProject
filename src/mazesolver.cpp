/*
   Line sensor on A0,A1,A2,A3,A4,A5,A6
   A0-left & A6 - right
*/

#include <Arduino.h>
#include <L298N.h>

const int startButton = 11;

bool l = 0;
bool r = 0;
bool s = 0;
bool u = 0;
int e = 0;
int paths = 0;

bool endFound = 0;

int FT = 70; // how much the bot moves forward before turning

int P, D, I, previousError, PIDvalue, error;
int lsp = 100;
int rsp = 100;

int lfspeed = 80; // change this for bot average speed

int turnspeed = 50;
float Kp = 0.05;
float Kd = 1;
float Ki = 0;

String str;

#define AIN1 4
#define BIN1 6
#define AIN2 3
#define BIN2 7
#define PWMA 9
#define PWMB 10
#define STBY 5

// these constants are used to allow you to make your motor configuration
// line up with function names like forward.  Value can be 1 or -1
const int offsetA = 1;
const int offsetB = 1;

// Initializing motors.  The library will allow you to initialize as many
// motors as you have memory for.  If you are using functions like forward
// that take 2 motors as arguements you can either write new functions or
// call the function more than once.

L298N motor1(AIN1, AIN2, PWMA);
L298N motor2(BIN1, BIN2, PWMB);

int minValues[8], maxValues[8], threshold[8];

void setup()
{
    Serial.begin(9600);
    pinMode(11, INPUT_PULLUP);
    pinMode(12, INPUT_PULLUP);
    pinMode(2, OUTPUT);
    pinMode(13, OUTPUT);

    red();
    delay(500);
    lightsoff();
}

void loop()
{

    while (digitalRead(11))
    {
    }
    delay(1000);
    calibrate();

    while (digitalRead(11))
    {
    }
    delay(500);

    while (endFound == 0)
    {
        linefollow();
        checknode();

        botstop();
        delay(100);

        reposition();
    }
    for (int m = 0; m < 4; m++)
    {
        str.replace("LUL", "S");
        str.replace("SUL", "R");
        str.replace("LUS", "R");
        str.replace("RUL", "U");
    }
    int endpos = str.indexOf('E');

    while (digitalRead(11))
    {
    }
    delay(5000);

    for (int i = 0; i <= endpos; i++)
    {
        char node = str.charAt(i);
        paths = 0;
        while (paths < 2)
        {
            linefollow();
            checknode();
            if (paths == 1)
            {
                botstop();
                delay(100);
                reposition();
            }
        }
        switch (node)
        {
        case 'L':
            botstop();
            delay(100);
            botleft();
            break;

        case 'S':
            break;

        case 'R':
            botstop();
            delay(100);
            botright();
            break;

        case 'E':
            red();
            botstop();
            delay(5000);
            break;
        } //_________end of switch
    }     //_________end of for loop
}

void calibrate()
{
    for (int i = 0; i < 7; i++)
    {
        minValues[i] = analogRead(i);
        maxValues[i] = analogRead(i);
    }

    for (int i = 0; i < 1500; i++)
    {
        motor1.setSpeed(30);
        motor1.forward();
        motor2.setSpeed(30);
        motor2.backward();

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
        Serial.print("   ");
    }
    Serial.println();

    motor1.stop();
    motor2.stop();

    red();
    delay(500);
    lightsoff();
}

void checknode()
{
    l = 0;
    r = 0;
    s = 0;
    u = 0;
    e = 0;
    paths = 0;

    // checks whethere bot is on node and the number of exits possible

    for (int i = 0; i < 50; i++)
    {
        if (analogRead(6) < threshold[6])
            r = 1;
        if (analogRead(0) < threshold[0])
            l = 1;
        if ((analogRead(0) > threshold[0] && (analogRead(6) > threshold[6]) && (analogRead(3) > threshold[3])))
        {
            u = 1;
        }
        if ((analogRead(3) < threshold[3]) && (analogRead(0) < threshold[0]) && (analogRead(6) < threshold[6]))
        {
            e = 1;
        }
    }

    if (u == 0)
    {
        for (int i = 0; i < FT; i++)
        {
            PID();
            if (analogRead(6) < threshold[6])
                r = 1;
            if (analogRead(0) < threshold[0])
                l = 1;
        }

        for (int i = 0; i < FT; i++)
        {
            PID();
            if (analogRead(3) < threshold[3])
                s = 1;
            if ((e == 1) && (analogRead(0) < threshold[0]) && (analogRead(6) < threshold[6]) && (analogRead(3) < threshold[3]))
                e = 2;
        }
        if (e == 2)
        {
            for (int i = 0; i < FT; i++)
            {
                PID();
            }
            if ((analogRead(0) < threshold[0]) && (analogRead(6) < threshold[6]) && (analogRead(3) < threshold[3]))
                e = 3;
        }
    }
    if (u == 1)
    {
        for (int i = 0; i < 5; i++)
        {
            botinchforward();
        }
    }

    paths = l + s + r;
}

void red()
{
    digitalWrite(2, LOW);
    digitalWrite(8, LOW);
    digitalWrite(13, HIGH); // RED
}

void lightsoff()
{
    digitalWrite(2, LOW);
    digitalWrite(8, LOW);
    digitalWrite(13, LOW);
}

void linefollow()
{
    paths = 0;
    while ((analogRead(0) > threshold[0]) && (analogRead(6) > threshold[6]) && (analogRead(3) < threshold[3]))
    {
        PID();
    }
}
void PID()
{
    int error = analogRead(2) - analogRead(4);

    P = error;
    I = I + error;
    D = error - previousError;

    PIDvalue = (Kp * P) + (Ki * I) + (Kd * D);
    previousError = error;

    lsp = lfspeed - PIDvalue;
    rsp = lfspeed + PIDvalue;

    if (lsp > 200)
    {
        lsp = 200;
    }
    if (lsp < 0)
    {
        lsp = 0;
    }
    if (rsp > 200)
    {
        rsp = 200;
    }
    if (rsp < 0)
    {
        rsp = 0;
    }

    motor1.setSpeed(rsp);
    motor1.forward();
    motor2.setSpeed(lsp);
    motor2.forward();
}

void reposition()
{
    if (e == 3)
    {
        str += 'E';
        endFound = 1;
        red();
        botstop();
        delay(2000);
        lightsoff();
    }
    else if (l == 1)
    {
        if (paths > 1)
            str += 'L';
        botleft(); // take left
    }

    else if (s == 1)
    {
        if (paths > 1)
            str += 'S';
    }
    else if (r == 1)
    {
        if (paths > 1)
            str += 'R';
        botright(); // take right
    }

    else if (u == 1)
    {
        str += 'U';
        botuturn(); // take left
    }
}

void botleft()
{
    motor1.setSpeed(turnspeed);
    motor1.backward();
    motor2.setSpeed(turnspeed);
    motor2.forward();
    delay(250);
    while (analogRead(3) > threshold[3])
    {
        motor1.setSpeed(turnspeed);
        motor1.backward();
        motor2.setSpeed(turnspeed);
        motor2.forward();
    }
    motor1.stop();
    motor2.stop();
    delay(50);
}

void botright()
{
    motor1.setSpeed(turnspeed);
    motor1.forward();
    motor2.setSpeed(turnspeed);
    motor2.backward();
    delay(250);
    while (analogRead(3) > threshold[3])
    {
        motor1.setSpeed(turnspeed);
        motor1.forward();
        motor2.setSpeed(turnspeed);
        motor2.backward();
    }
    motor1.stop();
    motor2.stop();
    delay(50);
}

void botstraight()
{
    motor1.setSpeed(lfspeed);
    motor1.forward();
    motor2.setSpeed(lfspeed);
    motor2.forward();
}

void botinchforward()
{
    motor1.setSpeed(turnspeed);
    motor1.forward();
    motor2.setSpeed(turnspeed);
    motor2.forward();
    delay(10);
}
void botstop()
{
    motor1.stop();
    motor2.stop();
}
void botuturn()
{
    motor1.setSpeed(turnspeed);
    motor1.backward();
    motor2.setSpeed(turnspeed);
    motor2.forward();
    delay(300);
    while (analogRead(3) > threshold[3])
    {
        motor1.setSpeed(turnspeed);
        motor1.backward();
        motor2.setSpeed(turnspeed);
        motor2.forward();
    }
    motor1.stop();
    motor2.stop();
    delay(50);
}
