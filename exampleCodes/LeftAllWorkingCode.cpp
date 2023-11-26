
#include <MOTORS.h>
#include <ARRAY.h>

MOTORS motors(3,5,6,7,8,9);


ARRAY array(A0, A1, A2, A3, A4);
// Global constants for sensor values
const unsigned int black = 0;
const unsigned int white = 1;

// Global constant for turning speed
const unsigned int turning_speed = 70;

// Function to check if a sensor is on black
bool isBlack(int sensor) {
  return (sensor == black);
}

// Function to check if a sensor is on white
bool isWhite(int sensor) {
  return (sensor == white);
}
void printInfo(int sensor1, int sensor2, int sensor3, int sensor4, int sensor5) {
  Serial.print(sensor1);
  Serial.print(" ");
  Serial.print(sensor2);
  Serial.print(" ");
  Serial.print(sensor3);
  Serial.print(" ");
  Serial.print(sensor4);
  Serial.print(" ");
  Serial.println(sensor5);
}

void setup() {
  Serial.begin(9600);


  pinMode(A0, INPUT);  // Extreme left sensor
  pinMode(A1, INPUT);  // Left sensor
  pinMode(A2, INPUT);  // Middle sensor
  pinMode(A3, INPUT);  // Right sensor
  pinMode(A4, INPUT);  // Extreme right sensor
}

void loop() {
  // Read sensor values
  int sensor1 = digitalRead(A0);  // Extreme left sensor
  int sensor2 = digitalRead(A1);  // Left sensor
  int sensor3 = digitalRead(A2);  // Middle sensor
  int sensor4 = digitalRead(A3);  // Right sensor
  int sensor5 = digitalRead(A4);  // Extreme right sensor

  if (isBlack(sensor1)) {
    // If extreme left sensor is on black, turn left
    motors.turnLeft(turning_speed);
  } else {
    if (isBlack(sensor2)) {
      // If left sensor is on black, turn left
      motors.turnLeft(turning_speed);
    } else if (isBlack(sensor4)) {
      // If right sensor is on black, turn right
      motors.turnRight(turning_speed);
    } else if (isWhite(sensor1) && isBlack(sensor3)) {
      // If extreme left and middle sensors are on white, move forward
      motors.forward(80);
    } else if (isWhite(sensor1) && isWhite(sensor2) && isWhite(sensor3) && isWhite(sensor4) && isBlack(sensor5)) {
      // If gap detected, turn in a circle
      motors.turnAround(80);
    } else if (isWhite(sensor1) && isWhite(sensor3) && isBlack(sensor5)) {
      // If extreme right sensor is on black, turn right
      motors.turnRight(turning_speed);
    }
  }
  // motors.forward(80);
}