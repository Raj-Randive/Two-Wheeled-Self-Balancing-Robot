// Required Libraries
#include <Wire.h>
#include <MPU6050.h>
#include <AccelStepper.h>

// MPU6050 Constants
const int MPU_ADDR = 0x68; // MPU6050 I2C address

// Motor Constants
const int MOTOR1_STEP_PIN = 6;  // Step pin for motor 1
const int MOTOR1_DIR_PIN = 7;   // Direction pin for motor 1
const int MOTOR2_STEP_PIN = 8;  // Step pin for motor 2
const int MOTOR2_DIR_PIN = 9;   // Direction pin for motor 2

// Motor Configurations
const float MOTOR1_GEAR_RATIO = 1.0;   // Gear ratio for motor 1
const float MOTOR2_GEAR_RATIO = 1.0;   // Gear ratio for motor 2
const int MOTOR_STEPS_PER_REV = 200;   // Number of steps per motor revolution

// Balancing Constants
const float K_P = 30.0;    // Proportional gain
const float K_D = 10.0;    // Derivative gain
const float K_I = 0.0;     // Integral gain

// MPU6050 Variables
MPU6050 mpu;
int16_t accelX, accelY, accelZ;
int16_t gyroX, gyroY, gyroZ;
float angle;

// Motor Objects
AccelStepper motor1(AccelStepper::DRIVER, MOTOR1_STEP_PIN, MOTOR1_DIR_PIN);
AccelStepper motor2(AccelStepper::DRIVER, MOTOR2_STEP_PIN, MOTOR2_DIR_PIN);

// PID Variables
float previousError = 0.0;
float integral = 0.0;

// Target Angle
float targetAngle = 0.0;

// Setup Function
void setup() {
  Serial.begin(9600);

  // Initialize MPU6050
  Wire.begin();
  mpu.initialize();
  mpu.setFullScaleAccelRange(0);

  // Set motor speed and acceleration
  motor1.setMaxSpeed(1000);
  motor1.setAcceleration(15000);
  motor2.setMaxSpeed(1000);
  motor2.setAcceleration(15000);

  // Calibrate MPU6050
  calibrateMPU6050();

  // Set initial motor positions
  motor1.setCurrentPosition(0);
  motor2.setCurrentPosition(0);
}

// Loop Function
void loop() {
  // Read MPU6050 data
  readMPU6050Data();

  // Calculate angle using complementary filter
  float angleAccel = atan2(accelY, accelZ) * (180.0 / PI);
  angle = 0.98 * (angle + gyroX * 0.01) + 0.02 * angleAccel;

  // Calculate error and PID terms
  float error = targetAngle - angle;
  float output = K_P * error + K_D * (error - previousError) + K_I * integral;

  // Update previous error and integral
  previousError = error;
  integral += error;

  // Apply control output to motors
  motor1.setSpeed(output / (MOTOR1_GEAR_RATIO * MOTOR_STEPS_PER_REV));
  motor1.runSpeedToPosition();
  motor2.setSpeed(output / (MOTOR2_GEAR_RATIO * MOTOR_STEPS_PER_REV));
  motor2.runSpeedToPosition();
}

// MPU6050 Calibration
void calibrateMPU6050() {
  int16_t accX = 0, accY = 0, accZ = 0;
  int16_t gyroX = 0, gyroY = 0, gyroZ = 0;

  // Read and discard initial values
  for (int i = 0; i < 2000; i++) {
    mpu.getMotion6(&accX, &accY, &accZ, &gyroX, &gyroY, &gyroZ);
  }

  // Compute average offsets
  int32_t accXOffset = 0, accYOffset = 0, accZOffset = 0;
  int32_t gyroXOffset = 0, gyroYOffset = 0, gyroZOffset = 0;
  for (int i = 0; i < 2000; i++) {
    mpu.getMotion6(&accX, &accY, &accZ, &gyroX, &gyroY, &gyroZ);
    accXOffset += accX;
    accYOffset += accY;
    accZOffset += accZ;
    gyroXOffset += gyroX;
    gyroYOffset += gyroY;
    gyroZOffset += gyroZ;
    delay(3);
  }
  accXOffset /= 2000;
  accYOffset /= 2000;
  accZOffset /= 2000;
  gyroXOffset /= 2000;
  gyroYOffset /= 2000;
  gyroZOffset /= 2000;

  // Set MPU6050 offsets
  mpu.setXAccelOffset(-accXOffset);
  mpu.setYAccelOffset(-accYOffset);
  mpu.setZAccelOffset(16384 - accZOffset);
  mpu.setXGyroOffset(-gyroXOffset);
  mpu.setYGyroOffset(-gyroYOffset);
  mpu.setZGyroOffset(-gyroZOffset);
}

// Read MPU6050 Data
void readMPU6050Data() {
  mpu.getMotion6(&accelX, &accelY, &accelZ, &gyroX, &gyroY, &gyroZ);
}
