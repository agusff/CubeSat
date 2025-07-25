#ifndef MPU9250_H
#define MPU9250_H

#include <MPU9250_asukiaaa.h>

class MPU9250Sensor {
  private:
    int sda, scl;
    MPU9250_asukiaaa mpu;

    float roll = 0, pitch = 0, yaw = 0;
    float alpha = 0.9;
    float dt;
    unsigned long lastTime = 0;

    float gyroXoffset = 0, gyroYoffset = 0, gyroZoffset = 0;
    void computeOrientation();

  public:
    MPU9250Sensor(uint8_t sdaPin, uint8_t sclPin);

    bool begin();
    bool isConnected();
    void update();
    void calibrateGyro();

    float getRoll();
    float getPitch();
    float getYaw();

    float getGyroX();
    float getGyroY();
    float getGyroZ();
};

#endif