#include "BME680.h"

void BME680Sensor::setWire(TwoWire* wire) {
  _wire = wire;
}

bool BME680Sensor::begin() {
  if (!_wire) return false; 

  if (!bme.begin(0x77, _wire)) {  
    return false;
  }

  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150);
  return true;
}

bool BME680Sensor::readData() {
  if (!isConnected()) return false; // Agregado para evitar bloqueo

  if (!bme.performReading()) {
    return false;
  }

  temp = bme.temperature;
  press = bme.pressure;
  hum = bme.humidity;
  return true;
}

bool BME680Sensor::isConnected() {
  _wire->beginTransmission(0x77);
  return (_wire->endTransmission() == 0);
}

float BME680Sensor::getTemp() { return temp; }
float BME680Sensor::getPress() { return press; }
float BME680Sensor::getHum() { return hum; }