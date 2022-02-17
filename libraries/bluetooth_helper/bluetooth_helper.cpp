#include "bluetooth_helper.h"

extern BluetoothSerial s;

extern float a[2][N_COORDINATES] = {0};

void bluetooth_setup(){
    s.begin("ArmWarmer");
}

void bluetooth_read_coordinates(){
    if(s.available()){
      for(int lat = 0;lat < N_COORDINATES ; lat++){ 
        Serial.println("Coordinates [lat] [lon]:");  
          for(int lon = 0; lon < N_DIMENSIONS; lon++){
              a[lat][lon] = s.parseFloat(); //Read float from serial---------------------------------
              Serial.println(a[lat][lon],6);
          }
        Serial.println("");
      }
    }
}

void bluetooth_read_to_eeprom(){
}
