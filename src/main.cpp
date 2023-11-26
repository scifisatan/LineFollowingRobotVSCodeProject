#include <Arduino.h>

void setup()
{
    Serial.begin(9600);
}

void loop()
{

    if (Serial.available())
    {                                 // if there is data comming
        char command = Serial.read(); // read string until meet newline character

        Serial.println(command); // print what was read to serial port
    }
    delay(1000);
}