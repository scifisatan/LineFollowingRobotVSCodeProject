// Required Libraries
#include <MOTORS.h>

// Define motor control pins
// Left motor
const unsigned int EN_A = 3;
const unsigned int IN1_A = 6;
const unsigned int IN2_A = 5;

// Right motor
const unsigned int IN1_B = 8;
const unsigned int IN2_B = 7;
const unsigned int EN_B = 9;

// Global constants for sensor values
const unsigned int black = 0;
const unsigned int white = 1;

// Global constant for turning speed
const unsigned int turning_speed = 70;

// Initialize both motors
MOTORS motors(EN_A, IN1_A, IN2_A, EN_B, IN1_B, IN2_B);

void setup() {
  // Set up serial communication
  Serial.begin(9600);

  // Wait for Serial Monitor to be opened
  while (!Serial) {
    // Do nothing
  }

  // Set up sensor pins
  pinMode(A0, INPUT);  // Extreme left sensor
  pinMode(A1, INPUT);  // Left sensor
  pinMode(A2, INPUT);  // Middle sensor
  pinMode(A3, INPUT);  // Right sensor
  pinMode(A4, INPUT);  // Extreme right sensor

}

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

void loop() {
  // Read sensor values
  int sensor1 = digitalRead(A1);  // Extreme left sensor
  int sensor2 = digitalRead(A2);  // Left sensor
  int sensor3 = digitalRead(A3);  // Middle sensor
  int sensor4 = digitalRead(A4);  // Right sensor
  int sensor5 = digitalRead(A5);  // Extreme right sensor

  // printInfo(sensor1, sensor2, sensor3, sensor4, sensor5);
  // Serial.println("going forward");
  // motors.forward();
  // delay(1000);
  // Serial.println("going backward");
  // motors.backward();
  // delay(1000);
  // turnLeft();
  // delay(1000);
  // turnRight();
  // delay(1000);
  // turnCircle();
  // delay(1000);

  if (isBlack(sensor1)) {
    // If extreme left sensor is on black, turn left
    turnLeft();
  } else {
    if (isBlack(sensor2)) {
      // If left sensor is on black, turn left
      turnLeft();
    } else if (isBlack(sensor4)) {
      // If right sensor is on black, turn right
      turnRight();
    } else if (isWhite(sensor1) && isBlack(sensor3)) {
      // If extreme left and middle sensors are on white, move forward
      motors.forward();
    } else if (isWhite(sensor1) && isWhite(sensor2) && isWhite(sensor3) && isWhite(sensor4) && isBlack(sensor5)) {
      // If gap detected, turn in a circle
      turnCircle();
    } else if (isWhite(sensor1) && isWhite(sensor3) && isBlack(sensor5)) {
      // If extreme right sensor is on black, turn right
      turnRight();
    }
  }
}
