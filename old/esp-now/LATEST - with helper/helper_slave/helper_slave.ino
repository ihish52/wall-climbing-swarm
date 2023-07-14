 #include "esp-now_helper.h"
 
void setup() {
  // put your setup code here, to run once:
  setup_esp_now_slave();
}

void loop() {
  // put your main code here, to run repeatedly:
  slave_send(1,5,6,7,8);
}
