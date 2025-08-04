#include "BME680.h"
#include "MPU9250.h"
#include "MSD.h"
#include "Wire.h"
#include "HT_st7735.h"
#include "HT_TinyGPS++.h"

#define BTN 46            //Pin para conectar pulsador con resistencia PullDown
#define VGNSS_CTRL 3
#define SDA_Pin 21        //I2C para sensores
#define SCL_Pin 37
#define CS_PIN 36         //Conexion al modulo SD
#define SCK_PIN 12
#define MISO_PIN 11
#define MOSI_PIN 9


TinyGPSPlus GPS;
HT_st7735 st7735;

//Objetos de las clases
BME680Sensor BME680;
MPU9250Sensor MPU9250;
SD_Card Sd(CS_PIN, SCK_PIN, MISO_PIN, MOSI_PIN);

//Bandera de verificacion de conexion
bool bme680_ok = false;
bool mpu9250_ok = false;
bool sd_ok = false;
bool gps_ok = false;

//Variables del loop
unsigned long PrevTime1 = 0;
bool missionActive = false;
bool btnLastState = LOW;

//Variables de datos
float temp, press, hum;
float roll, pitch, yaw, gx, gy, gz;
String hour;

void setup() {
  delay(100);
  Serial.begin(115200);

  Wire.begin(SDA_Pin, SCL_Pin); 
  Wire.setClock(400000);

  pinMode(VGNSS_CTRL,OUTPUT);
	digitalWrite(VGNSS_CTRL,HIGH);
	Serial1.begin(115200,SERIAL_8N1,33,34);

  pinMode(BTN, INPUT);

  st7735.st7735_init();
  st7735.st7735_fill_screen(ST7735_BLACK);
	st7735.st7735_write_str(0, 0, (String)"Inicializando...");
  delay(300);
  st7735.st7735_fill_screen(ST7735_BLACK);

  //Inicializacion I2C de sensores
  BME680.setWire(&Wire);
  MPU9250.setWire(&Wire);
  
  //Inicializacion de sensores
  if(!BME680.begin()){
    Serial.println("Error al iniciar BME680");
	  st7735.st7735_write_str(0, 0, (String)"BME680 Error");
   }else{
    bme680_ok = true;
    Serial.println("BME680 Listo. Esperando comienzo de mision");
	  st7735.st7735_write_str(0, 0, (String)"BME680 Ready");
   } 

  if(!MPU9250.begin()){
    Serial.println("Error al iniciar MPU9250");
	  st7735.st7735_write_str(0, 20, (String)"MPU9250 Error");
  }else{
    st7735.st7735_write_str(0, 20, (String)"MPU9250 ...");
    MPU9250.calibrateGyro();
    mpu9250_ok = true;
    Serial.println("MPU9250 Listo. Esperando comienzo de mision");
	  st7735.st7735_write_str(0, 20, (String)"MPU9250 Ready");
   }

   if(!Sd.begin()){
    Serial.println("Error al iniciar SD Card");
    st7735.st7735_write_str(0, 40, (String)"SD Error");
   }else{
    sd_ok = true;
    Serial.println("SD Card Listo. Esperando comienzo de mision");
	  st7735.st7735_write_str(0, 40, (String)"SD Ready");
   }

   while(!gps_ok){
      Serial.println("Esperando GPS");
      st7735.st7735_write_str(0, 60, (String)"GPS ...");  
        // Procesar datos del GPS
      while (Serial1.available()) {
        GPS.encode(Serial1.read());
      }
      if(GPS.time.isValid() && GPS.time.hour() != 0 && GPS.time.minute() != 0 && GPS.time.second() != 0){
        gps_ok = true;
        Serial.println("GPS Listo. Esperando comienzo de mision");
        st7735.st7735_write_str(0, 60, (String)"GPS Ready"); 
      }
    delay(100);
   }
  };

//Funcion para finalizar la mision
void stopRequested(){
  Sd.finish();
  missionActive = false;
  Serial.println("Misión finalizada correctamente.");
  st7735.st7735_fill_screen(ST7735_BLACK);
	st7735.st7735_write_str(0, 0, (String)"Mision Finish");
}

//Verifica si los sensores estan conectados
void status(bool &bme680_ok, bool &mpu9250, bool &sd_ok){
  bme680_ok = BME680.isConnected();
  mpu9250_ok = MPU9250.isConnected();
  sd_ok = Sd.status();
}

String utc3(){     // Convierte la hora a UTC-3
  if (!GPS.time.isValid()) 
    return "No hour";

  String hora = "00:00:00";
  int h = (GPS.time.hour() - 3 + 24) % 24;  
  hora = String(h < 10 ? "0" : "") + String(h) + ":" +
          String(GPS.time.minute() < 10 ? "0" : "") + String(GPS.time.minute()) + ":" +
          String(GPS.time.second() < 10 ? "0" : "") + String(GPS.time.second());
  return hora;
}

void loop() {

  status(bme680_ok, mpu9250_ok, sd_ok);    //Verifica sensores
  bool btnState = digitalRead(BTN);
  
  if (btnState == HIGH && btnLastState == LOW) {      // Se presionó el botón
    if (!missionActive) {
      missionActive = true;
      Serial.println("Misión iniciada.");
      st7735.st7735_fill_screen(ST7735_BLACK);
	    st7735.st7735_write_str(0, 0, (String)"Mision Init");
      delay(300);
      st7735.st7735_fill_screen(ST7735_BLACK);
    } else {
      stopRequested();  // Señal de detener misión
    }
  }

  btnLastState = btnState;

  if (missionActive) {
    // Procesa datos del GPS 
    while (Serial1.available()) {
      GPS.encode(Serial1.read());
    }

    //Funcionamiento del BME680
    if(bme680_ok){
      BME680.readData();
      if(bme680_ok){ st7735.st7735_write_str(0, 0, (String)"BME680: ok");

      temp = BME680.getTemp();
      press = BME680.getPress();
      hum = BME680.getHum();

    }else{
      st7735.st7735_write_str(0, 0, (String)"BME680: Er");

      temp = -1;
      press = -1;
      hum = -1;
    }
    //Funcionamiento del MPU9250
    if(mpu9250_ok){
      MPU9250.update(); 
      st7735.st7735_write_str(0, 20, (String)"MPU9250: ok");

      roll = MPU9250.getRoll();
      pitch = MPU9250.getPitch();
      yaw = MPU9250.getYaw();
      gx = MPU9250.getGyroX();
      gy = MPU9250.getGyroY();
      gz = MPU9250.getGyroZ();

    }else{
      st7735.st7735_write_str(0, 20, (String)"MPU9250: Er");

      roll = -1;
      pitch = -1;
      yaw = -1;
      gx = -1;
      gy = -1;
      gz = -1;

    }

    if (GPS.time.isValid() && GPS.time.age() < 2000) {
      // Obtiene la timestamp GPS
      hour = utc3();
      st7735.st7735_write_str(0, 60, (String)hour);
    }

    if(sd_ok){
      st7735.st7735_write_str(0, 40, (String)"SD: ok");
      String sensorData = 
        String(hour) + ";" +
        String(temp, 2) + ";" + 
        String(press, 2) + ";" + 
        String(hum, 2) + ";" + 
        String(roll, 2) + ";" + 
        String(pitch, 2) + ";" + 
        String(yaw, 2) + ";" +
        String(gx, 2) + ";" +
        String(gy, 2) + ";" +
        String(gz, 2);
      if(Sd.log(sensorData)){
        Serial.print("Datos Guardados");
      }else {Serial.print("Error al Guardar");}
    }else {
      st7735.st7735_write_str(0, 40, (String)"SD: Er");
    }

  Serial.print("Hour: ");
  Serial. print(hour);
  Serial.print(", Temp: ");
  Serial. print(temp);
  Serial.print(", Press: ");
  Serial. print(press);
  Serial.print(", Hum%: ");
  Serial. print(hum);
  Serial.print(", Roll: ");
  Serial. print(roll);
  Serial.print(", Pitch: ");
  Serial. print(pitch);
  Serial.print(", Yaw: ");
  Serial. print(yaw);
  Serial.print(", Gx: ");
  Serial. print(gx);
  Serial.print(", Gy: ");
  Serial. print(gy);
  Serial.print(", Gz: ");
  Serial. println(gz);
    }
  }
}
