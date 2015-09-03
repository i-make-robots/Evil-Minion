#include "config.h"

#include <EEPROM.h>




//------------------------------------------------------------------------------
// from http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1234477290/3
void EEPROM_writeLong(int ee, long value) {
  byte* p = (byte*)(void*)&value;
  for (int i = 0; i < sizeof(value); i++)
  EEPROM.write(ee++, *p++);
}


//------------------------------------------------------------------------------
// from http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1234477290/3
float EEPROM_readLong(int ee) {
  long value = 0;
  byte* p = (byte*)(void*)&value;
  for (int i = 0; i < sizeof(value); i++)
  *p++ = EEPROM.read(ee++);
  return value;
}


//------------------------------------------------------------------------------
void LoadConfig() {
  char version_number=EEPROM.read(ADDR_VERSION);
  if(version_number!=EEPROM_VERSION) {
    // If not the current EEPROM_VERSION or the EEPROM_VERSION is sullied (i.e. unknown data)
    // Update the version number
    EEPROM.write(ADDR_VERSION,EEPROM_VERSION);
    // Update robot uuid
    robot_uid=0;
    SaveUID();
  }
  
  if(version_number==1) {
  } else {
    // Code should not get here if it does we should display some meaningful error message
    Serial.println(F("An Error Occurred during LoadConfig"));
  }
}


//------------------------------------------------------------------------------
void SaveUID() {
  EEPROM_writeLong(ADDR_GUID,robot_uid);
}

