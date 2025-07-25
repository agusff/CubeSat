#include "MPU9250.h"
#include <Wire.h>
#include <Arduino.h>
#include <math.h>

MPU9250Sensor::MPU9250Sensor(uint8_t sdaPin, uint8_t sclPin) {
  sda = sdaPin;
  scl = sclPin;
}

bool MPU9250Sensor::begin() {
  Wire.begin(sda, scl);
  mpu.setWire(&Wire);

  uint8_t id;
  if (mpu.readId(&id) != 0) return false;

  mpu.beginAccel();
  mpu.beginGyro();

  if (mpu.accelUpdate() != 0 || mpu.gyroUpdate() != 0) return false;

  calibrateGyro();
  lastTime = micros();
  return true;
}

bool MPU9250Sensor::isConnected() {
  Wire.beginTransmission(0x68);
  return Wire.endTransmission() == 0;
}

void MPU9250Sensor::calibrateGyro() {
  const int samples = 3000;
  float sumX = 0, sumY = 0, sumZ = 0;

  for (int i = 0; i < samples; i++) {
    mpu.gyroUpdate();
    sumX += mpu.gyroX();
    sumY += mpu.gyroY();
    sumZ += mpu.gyroZ();
    delay(3);
  }

  gyroXoffset = sumX / samples;
  gyroYoffset = sumY / samples;
  gyroZoffset = sumZ / samples;
}

void MPU9250Sensor::update() {
  mpu.accelUpdate();
  mpu.gyroUpdate();

  unsigned long currentTime = micros();
  dt = (currentTime - lastTime) / 1000000.0;
  lastTime = currentTime;

  computeOrientation();
}

void MPU9250Sensor::computeOrientation() {
  float ax = mpu.accelX();
  float ay = mpu.accelY();
  float az = mpu.accelZ();

  float gx = mpu.gyroX() - gyroXoffset;
  float gy = mpu.gyroY() - gyroYoffset;
  float gz = mpu.gyroZ() - gyroZoffset;

  float accelRoll = atan2(ay, az) * 180 / PI;
  float accelPitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180 / PI;

  roll += gx * dt;
  pitch += gy * dt;
  yaw += gz * dt;

  roll = alpha * roll + (1 - alpha) * accelRoll;
  pitch = alpha * pitch + (1 - alpha) * accelPitch;
}

float MPU9250Sensor::getRoll()  { return roll; }
float MPU9250Sensor::getPitch() { return pitch; }
float MPU9250Sensor::getYaw()   { return yaw; }

float MPU9250Sensor::getGyroX() { return mpu.gyroX() - gyroXoffset; }
float MPU9250Sensor::getGyroY() { return mpu.gyroY() - gyroYoffset; }
float MPU9250Sensor::getGyroZ() { return mpu.gyroZ() - gyroZoffset; }
