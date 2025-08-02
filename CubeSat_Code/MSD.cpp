#include <iterator>
#include "MSD.h"

SD_Card::SD_Card(uint8_t csPin, uint8_t sckPin, uint8_t misoPin, uint8_t mosiPin ){
  CS_Pin = csPin;
  SCK_Pin = sckPin;
  MISO_Pin = misoPin;
  MOSI_Pin = mosiPin;
} 

bool SD_Card::begin() {
  SPI.begin(SCK_Pin, MISO_Pin, MOSI_Pin, CS_Pin);
  if (!SD.begin(CS_Pin)) {
    SD_Ready = false;
    return false;
  }

  SD_File = SD.open("/datos.csv", FILE_APPEND);
  if (!SD_File) {
    SD_Ready = false;
    return false;
  }

  SD_Ready = true;
  return true;
}

bool SD_Card::log(String data) {
  if (!SD_Ready || !SD_File)
   return false;
  
  SD_File.println(data);
  return true;
}

bool SD_Card::status() {
  return SD_Ready && SD_File;
}

void SD_Card::finish() {
  if (SD_File) {
    SD_File.flush();
    SD_File.close();
  }
}
