#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

void setup() {
  Serial.begin(9600);
  
  Wire.begin();
  
  mpu.initialize();
  
  Serial.println("MPU6050 connection successful");
  Serial.println("Calibrating gyro... Please wait.");
  mpu.CalibrateGyro();
  Serial.println("Gyro calibration complete.");
  
  delay(1000);
}

void loop() {
  Vector3f accelerometerData = mpu.getAcceleration();
  Vector3f gyroscopeData = mpu.getRotation();
  
  Serial.print("Accelerometer:\t");
  Serial.print("X = ");
  Serial.print(accelerometerData.x);
  Serial.print("\tY = ");
  Serial.print(accelerometerData.y);
  Serial.print("\tZ = ");
  Serial.println(accelerometerData.z);
  
  Serial.print("Gyroscope:\t");
  Serial.print("X = ");
  Serial.print(gyroscopeData.x);
  Serial.print("\tY = ");
  Serial.print(gyroscopeData.y);
  Serial.print("\tZ = ");
  Serial.println(gyroscopeData.z);
  
  delay(1000);
}
