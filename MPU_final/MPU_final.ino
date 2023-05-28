#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "math.h"

MPU6050 mpu;

// Define motor control pins
const int enaPin = 13;   // Enable pin for Motor A
const int in1Pin = 12;   // Input pin 1 for Motor A
const int in2Pin = 11;   // Input pin 2 for Motor A
const int enbPin = 8;    // Enable pin for Motor B
const int in3Pin = 10;   // Input pin 1 for Motor B
const int in4Pin = 9;    // Input pin 2 for Motor B

int16_t accX, accZ;
float accAngle;

void forward(int motorSpeed) {
  analogWrite(enaPin, motorSpeed);
  analogWrite(enbPin, motorSpeed);
  digitalWrite(in1Pin, HIGH);
  digitalWrite(in2Pin, LOW);
  digitalWrite(in3Pin, HIGH);
  digitalWrite(in4Pin, LOW);
}

void backward(int motorSpeed) {
  analogWrite(enaPin, motorSpeed);
  analogWrite(enbPin, motorSpeed);
  digitalWrite(in1Pin, LOW);
  digitalWrite(in2Pin, HIGH);
  digitalWrite(in3Pin, LOW);
  digitalWrite(in4Pin, HIGH);
}

void setup() {  
  mpu.initialize();
  Serial.begin(9600);
  // Initialize motor control pins
  pinMode(enaPin, OUTPUT);
  pinMode(in1Pin, OUTPUT);
  pinMode(in2Pin, OUTPUT);
  pinMode(enbPin, OUTPUT);
  pinMode(in3Pin, OUTPUT);
  pinMode(in4Pin, OUTPUT);

  digitalWrite(in1Pin, LOW);
  digitalWrite(in2Pin, LOW);
  digitalWrite(in3Pin, LOW);
  digitalWrite(in4Pin, LOW);
  Wire.begin();
}

void loop() {  
  accZ = mpu.getAccelerationZ();
  accX = mpu.getAccelerationX();
   
  accAngle = atan2(accX, accZ) * RAD_TO_DEG;
  
  if (!isnan(accAngle)) {
    Serial.println(accAngle);
    int motorSpeed = abs(accAngle);
    
    if (accAngle > 0) {
      Serial.print(accAngle);
      forward(motorSpeed);
    } 
    else {
      Serial.print(accAngle);
      backward(motorSpeed);
    }
    
    delay(100);
  }
}
