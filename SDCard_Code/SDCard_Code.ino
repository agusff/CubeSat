#include <SPI.h>
#include <SD.h>
#include "HT_st7735.h"
#include "HT_TinyGPS++.h"

#define CS_PIN 36              // Chip Select del módulo SD
#define DELAY 50          // 50 ms = 20 Hz
#define CHECKPOINT 5000   //Delay para forzar escritura de datos en la SD (5s)
#define btn1 46
#define VGNSS_CTRL 3

unsigned long PrevTime1 = 0;
unsigned long PrevTime2 = 0;
bool gpsInit = false;

File archivo;
TinyGPSPlus GPS;
HT_st7735 st7735;

void setup() {
  delay(100);
  Serial.begin(115200);

  pinMode(VGNSS_CTRL,OUTPUT);
	digitalWrite(VGNSS_CTRL,HIGH);
	Serial1.begin(115200,SERIAL_8N1,33,34);

  pinMode(btn1, INPUT);    //Pin con resistencia pulldown

  st7735.st7735_init();
  st7735.st7735_fill_screen(ST7735_BLACK);
	delay(100);
	st7735.st7735_write_str(0, 0, (String)"Inicializando...");

  // Inicializar SPI
  SPI.begin(12, 11, 9, CS_PIN); // SCK, MISO, MOSI, CS 
  
  if (!SD.begin(CS_PIN)) {
    Serial.println("No se pudo inicializar la tarjeta SD");
    st7735.st7735_fill_screen(ST7735_BLACK);
	  st7735.st7735_write_str(0, 0, (String)"Error en la SD");
    while (1);
  }

  // Abrir archivo para escritura
  archivo = SD.open("/datos.csv", FILE_APPEND);
  if (!archivo) {
    Serial.println("Error al abrir archivo");
    st7735.st7735_fill_screen(ST7735_BLACK);
	  st7735.st7735_write_str(0, 0, (String)"Error Archivo");
    while (1);
  }

  // Escribir encabezado CSV
  archivo.println("hora,gyro_x,gyro_y,gyro_z");
  archivo.flush();
  Serial.println("Inicio de registro...");
  st7735.st7735_fill_screen(ST7735_BLACK);
	st7735.st7735_write_str(0, 0, (String)"Inicio de");
	st7735.st7735_write_str(0, 20, (String)"registro");
  delay(500);
  st7735.st7735_fill_screen(ST7735_BLACK);


}

void safeFinish() {             //Funcion para dar un cierre seguro, cierra el archivo
  archivo.flush();
  archivo.close();
  Serial.println("Mision Finalizada");
  st7735.st7735_fill_screen(ST7735_BLACK);
	st7735.st7735_write_str(0, 0, (String)"Mision ");
	st7735.st7735_write_str(0, 20, (String)"Finalizada");
  while(1);
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

  unsigned long ActTime = millis();

  // Procesar datos del GPS
  while (Serial1.available()) {
    GPS.encode(Serial1.read());
  }

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  if(!gpsInit && GPS.time.isValid() && GPS.time.hour() != 0 && GPS.time.minute() != 0 && GPS.time.second() != 0){       //Espera a que se inicie el gps
    gpsInit = true;
    Serial.println("GPS inicializado");
    st7735.st7735_fill_screen(ST7735_BLACK);
    st7735.st7735_write_str(0, 0, (String)"GPS ok");
    delay(1000);
  }

  if(!gpsInit){
    Serial.println("Esperando GPS");
    st7735.st7735_fill_screen(ST7735_BLACK);
    st7735.st7735_write_str(0, 0, (String)"Esperando GPS");
    delay(1000);
    return;
  }
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  
  if (ActTime - PrevTime1 >= DELAY) {   //Cada 50ms ocurre esto (realmente son aprox 125ms por el uso de la pantalla)
    PrevTime1 = ActTime;

    // Simulación de lecturas         
    float gx = random(-200, 200) / 10.0;
    float gy = random(-200, 200) / 10.0;
    float gz = random(-200, 200) / 10.0;

    if (GPS.time.isValid() && GPS.time.age() < 2000) {

    // Obtiene la timestamp GPS
    String hour = utc3();

    // Crea línea CSV
    String line = hour + "," +
                   String(gx, 2) + "," +
                   String(gy, 2) + "," +
                   String(gz, 2);

    // Guardar en SD
    archivo.println(line);

    if(ActTime - PrevTime2 >=CHECKPOINT){     //Forza escritura cada 5s
      PrevTime2 = ActTime;
      archivo.flush();  
    }
     
    Serial.println(line);

	  st7735.st7735_write_str(0, 0, (String)hour);
	  st7735.st7735_write_str(0, 20, (String)"Guardando...");
    Serial.println("Sats: "+ String(GPS.satellites.value()));   //prueba si detecta satelites (no necesario)
    st7735.st7735_write_str(0, 40, "Sats: "+ String(GPS.satellites.value()));
    }
  }

  if(digitalRead(btn1) == HIGH){  //Lectura del boton de apagado
    safeFinish();
  }
}