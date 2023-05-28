// Include the AccelStepper Library
#include <AccelStepper.h>

// Define pin connections
const int dirPin = 7;
const int stepPin = 6;

// Define motor interface type
#define motorInterfaceType 1

// Creates an instance
AccelStepper myStepper(motorInterfaceType, stepPin, dirPin);

void setup() {
  // set the maximum speed, acceleration factor,
  // initial speed, and the target position
  myStepper.setMaxSpeed(300);
  myStepper.setAcceleration(15000);
  myStepper.setSpeed(300);
  myStepper.moveTo(100);
}

void loop() {
  // Change direction once the motor reaches the target position
  
  // Set the direction to clockwise
 // myStepper.setDirection(1); // 1 indicates clockwise, 0 indicates counterclockwise

  // Move the motor one step
  myStepper.runSpeed();
}
