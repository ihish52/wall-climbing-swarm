 #include "esp-now_helper.h"

 //extern variables in the esp-now_helper library
float allxposi[5], allyposi[5], allerro[5], allheadi[5];
char x;
int g; 

void setup() {
  // put your setup code here, to run once:
  setup_esp_now_controller();
}

void loop() {
  // put your main code here, to run repeatedly:
  controller_send();
}
