 #include "esp-now_helper.h"


void setup() {
  // put your setup code here, to run once:
  setup_esp_now_controller();
}

void loop() {
  // put your main code here, to run repeatedly:
  controller_send();
}
