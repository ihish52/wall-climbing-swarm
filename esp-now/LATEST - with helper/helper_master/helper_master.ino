 #include "esp-now_helper.h"

void setup() {
  // put your setup code here, to run once:
  setup_esp_now_master();
}

void loop() {
  // put your main code here, to run repeatedly:
  master_send(1,2,3,4);
}
