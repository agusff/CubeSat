#ifndef MSD_H
#define MSD_H

#include <Arduino.h>
#include <SD.h>

class SD_Card {
  private:
    uint8_t CS_Pin;
    File SD_File;
    bool SD_Ready = false;
  
  public:
    SD_Card(uint8_t csPin);
    bool begin();
    bool log(String data);
    bool status();
    void finish();
};

#endif