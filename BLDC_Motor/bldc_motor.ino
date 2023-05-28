#include <Wire.h>
#include <MPU6050.h>

// Define motor control pins
const int enaPin = 13;   // Enable pin for Motor A
const int in1Pin = 12;   // Input pin 1 for Motor A
const int in2Pin = 11;   // Input pin 2 for Motor A
const int enbPin = 8;    // Enable pin for Motor B
const int in3Pin = 10;   // Input pin 1 for Motor B
const int in4Pin = 9;    // Input pin 2 for Motor B


// Define MPU6050 object
MPU6050 mpu;

void setup() {
  // Initialize serial communication
  Serial.begin(9600);

  // Initialize motor control pins
  pinMode(enaPin, OUTPUT);
  pinMode(in1Pin, OUTPUT);
  pinMode(in2Pin, OUTPUT);
  pinMode(enbPin, OUTPUT);
  pinMode(in3Pin, OUTPUT);
  pinMode(in4Pin, OUTPUT);

  // Initialize MPU6050
  Wire.begin();
  mpu.initialize();
}

int16_t accY, accZ;
float accAngle;

void loop() {
  accZ = mpu.getAccelerationZ();
  accY = mpu.getAccelerationY();
   
  accAngle = atan2(accY, accZ) * RAD_TO_DEG;
  // Control robot movements based on gyroscope X-axis value

  // Set motor speed
  int motorSpeed = abs(accAngle);

  if (accAngle > 0) {
    // Rotate robot to the right
    moveForward(motorSpeed);
  // } else if (accAngle < 0) {
  //   // Rotate robot to the left
  //   turnLeft();
  } else {
    // Move robot forward
    moveBackward(motorSpeed);
  }

  

  analogWrite(enaPin, motorSpeed);
  analogWrite(enbPin, motorSpeed);

  // Print gyroscope X-axis value
  Serial.print("Gyroscope X: ");
  Serial.println(accAngle);

  delay(100);
}

// Function to move the robot forward
void moveForward(int motorSpeed) {
  analogWrite(enaPin, motorSpeed);
  analogWrite(enbPin, motorSpeed);
  digitalWrite(in1Pin, HIGH);
  digitalWrite(in2Pin, LOW);
  digitalWrite(in3Pin, HIGH);
  digitalWrite(in4Pin, LOW);
}

// Function to move the robot backward
void moveBackward(int motorSpeed) {
  analogWrite(enaPin, motorSpeed);
  analogWrite(enbPin, motorSpeed);
  digitalWrite(in1Pin, LOW);
  digitalWrite(in2Pin, HIGH);
  digitalWrite(in3Pin, LOW);
  digitalWrite(in4Pin, HIGH);
}

// // Function to turn the robot to the left
// void turnLeft() {
//   digitalWrite(in1Pin, LOW);
//   digitalWrite(in2Pin, HIGH);
//   digitalWrite(in3Pin, HIGH);
//   digitalWrite(in4Pin, LOW);
// }

// // Function to turn the robot to the right
// void turnRight() {
//   digitalWrite(in1Pin, HIGH);
//   digitalWrite(in2Pin, LOW);
//   digitalWrite(in3Pin, LOW);
//   digitalWrite(in4Pin, HIGH);
// }