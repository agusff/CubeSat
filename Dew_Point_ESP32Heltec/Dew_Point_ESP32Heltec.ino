#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"  //libreria Sensor

// Pines I2C 
#define I2C_SDA 21
#define I2C_SCL 20

Adafruit_BME680 bme;  // I2C

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Wire.begin(I2C_SDA, I2C_SCL);  // Inicia I2C en pines establecidos

  // Inicializa el sensor BME680
  if (!bme.begin()) {
    Serial.println("Error al detectar BME680 ");    //Si no se logra inicializar entra no avanza el programa
    while (1);
  }

  // Configura el sensor
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // Temperatura y duración del calentador (para medición de gas)

  Serial.println("Sensor BME680 iniciado correctamente ");
}

float dew_point(float temp, float hum){     //Calculo del punto de rocio
  const float a = 17.62;
  const float b = 243.12;

  float alpha = (a * temp) / (b + temp) + log(hum / 100.0);   //Ec. Magnus Tetens
  float Td = (b * alpha) / (a - alpha);
  return Td;
}

void loop() {
  // Prepara una lectura
  if (!bme.performReading()) {
    Serial.println("Error al leer del BME680");
    return;
  }

  //Variables de la medicion del sensor
  float temp = bme.temperature;
  float press = bme.pressure;
  float hum = bme.humidity;
  float td = dew_point(temp, hum);
  
  //Salidas en el monitor serial
  Serial.print("Temperatura = ");
  Serial.print(temp);
  Serial.println(" °C");

  
  Serial.print("Presión = ");
  Serial.print(press / 100.0);
  Serial.println(" hPa");

  
  Serial.print("Humedad = ");
  Serial.print(hum);
  Serial.println(" %");

  
  Serial.print("Punto de Rocío = ");
  Serial.print(td);
  Serial.println(" °C");


  Serial.println();
  delay(2000);
}
