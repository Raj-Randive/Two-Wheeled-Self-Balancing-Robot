#include <AccelStepper.h>

// Define pin connections for motor 1
const int dirPin1 = 7;
const int stepPin1 = 6;

// Define pin connections for motor 2
const int dirPin2 = 9;
const int stepPin2 = 8;

// Define motor interface type
#define motorInterfaceType 1

// Define the distance to travel (in steps)
const long targetDistance = 2000;

// Creates instances for motor 1 and motor 2
AccelStepper stepper1(motorInterfaceType, stepPin1, dirPin1);
AccelStepper stepper2(motorInterfaceType, stepPin2, dirPin2);


const int maxspeed = 200;
const int acc = 17000;
const int speed = 200;

void setup() {
  // Set the maximum speed, acceleration factor,
  // and initial speed for motor 1
  stepper1.setMaxSpeed(maxspeed);
  stepper1.setAcceleration(acc);
  stepper1.setSpeed(speed);
  stepper1.moveTo(100);

  // Set the maximum speed, acceleration factor,
  // and initial speed for motor 2
  stepper2.setMaxSpeed(maxspeed);
  stepper2.setAcceleration(acc);
  stepper2.setSpeed(speed);
  stepper2.moveTo(100);
}

void loop() {
  // Check if both stepper motors have reached the target distance
 
  // Move both stepper motors one step
  stepper1.runSpeed();
  stepper2.runSpeed();

  // Uncomment the following line to add a delay between steps
  // delay(1);
}

