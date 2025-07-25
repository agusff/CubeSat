#ifndef BME680_H
#define BME680_H

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>

class BME680Sensor {
  private:
    uint8_t sda, scl;
    Adafruit_BME680 bme;

  public:
    BME680Sensor(uint8_t sdaPin, uint8_t sclPin);
    bool begin();
    bool readData(float &temperature, float &pressure, float &humidity);
    bool isConnected();
};

#endif