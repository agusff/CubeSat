#ifndef MSD_H
#define MSD_H

#include <Arduino.h>
#include <SPI.h>
#include <SD.h>

class SD_Card {
  private:
    uint8_t CS_Pin;
    uint8_t SCK_Pin;
    uint8_t MISO_Pin;
    uint8_t MOSI_Pin;
    File SD_File;
    bool SD_Ready = false;
  
  public:
    SD_Card(uint8_t csPin, uint8_t sckPin, uint8_t misoPin, uint8_t mosiPin);
    bool begin();
    bool log(String data);
    bool status();
    void finish();
};

#endif