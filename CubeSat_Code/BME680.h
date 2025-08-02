#ifndef BME680_H
#define BME680_H

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>

class BME680Sensor {
  private:
    uint8_t sda, scl;
    Adafruit_BME680 bme;
    TwoWire* _wire;

    float temp, press, hum;

  public:
    void setWire(TwoWire* wire);
    bool begin();
    bool readData();
    bool isConnected();

    float getTemp();
    float getPress();
    float getHum();
};

#endif