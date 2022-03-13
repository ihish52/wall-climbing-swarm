
#include "LED_controller.h"

LED_controller LED;

void setup() {
  Serial.begin(115200);

  LED.setup();

}

void loop() {
  //Mode = 0
  LED.modes = 0;

  for(uint8_t j = 0 ; j < 3; j++){
    for(uint8_t i = 0; i<= 100; i++){
      LED.input = i;
      LED.led_function(); 
      Serial.println(LED.input);
      LED.setPixels();
      delay(10);
    }

    for(uint8_t i = 100; i> 0; i--){
      LED.input = i;
      LED.led_function(); 
      Serial.println(LED.input);
      LED.setPixels();
      delay(10);
    }
  }
  LED.modes = 1; 
  for(uint8_t j = 0 ; j < 3; j++){
    for(uint16_t i = 0; i<= 359; i++){
      LED.input = i;
      LED.led_function(); 
      Serial.println(LED.input);
      LED.setPixels();
      delay(10);
    }
  }

  LED.modes = 2;
  for(float i = 0; i<= 99; i+=0.1){
      LED.input = i;
      LED.led_function(); 
      Serial.println(LED.input);
      LED.setPixels();
      delay(10);
  }
}
