#include "BME680.h"
#include "MPU9250.h"
#include "MSD.h"

#define BTN 46
#define SDA_BME680 21
#define SCL_BME680 33

BME680Sensor BME680(SDA_BME680, SCL_BME680);

bool missionActive = false;
bool btnLastState = LOW;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;
bool stopRequested = false;

void setup() {
  delay(100);
  Serial.begin(115200);

  pinMode(BTN, INPUT);
  
  if(!BME680.begin()){
    Serial.println("Error al iniciar BME680");
   }else Serial.println("BME680 Listo. Esperando comienzo de mision");
  };


void loop() {
  bool btnState = digitalRead(BTN);

  if (btnState != btnLastState) {
    lastDebounceTime = millis();
  }
  if ((millis() - lastDebounceTime) > 0) {
    if (btnState == HIGH && btnLastState == LOW) {
      // Se presionó el botón
      if (!missionActive) {
        missionActive = true;
        Serial.println("Misión iniciada.");
      } else {
        stopRequested = true;  // Señal de detener misión
        Serial.println("Detención solicitada...");
      }
    }
  }
  btnLastState = btnState;

  if (missionActive) {
    float temp, press, hum;

    BME680.readData(temp, press, hum);

    Serial.print("Temperatura: ");
    Serial. println(temp);
    Serial.print("Presion: ");
    Serial. println(press);
    Serial.print("Humedad %: ");
    Serial. println(hum);

    if (stopRequested) {
      missionActive = false;
      stopRequested = false;
      Serial.println("Misión finalizada correctamente.");
    }
  }
}
