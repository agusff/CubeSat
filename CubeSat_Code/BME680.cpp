#include "BME680.h"

BME680Sensor::BME680Sensor(uint8_t sdaPin, uint8_t sclPin){
  sda = sdaPin;
  scl = sclPin;
}

bool BME680Sensor::begin() {
  Wire.begin(sda, scl);
  if (!bme.begin()) {
    return false;
  }

  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150);
  return true;
}

bool BME680Sensor::readData(float &temperature, float &pressure, float &humidity) {
  if (!bme.performReading()) {
    return false;
  }

  temperature = bme.temperature;
  pressure = bme.pressure;
  humidity = bme.humidity;
  return true;
}

bool BME680Sensor::isConnected() {
  Wire.beginTransmission(0x76); // Dirección I2C del BME680, puede ser 0x77 también.
  return (Wire.endTransmission() == 0);
}