 #include "esp-now_helper.h"

float allxposi[5], allyposi[5], allerro[5], allheadi[5] = {-1,-1,-1,-1,-1};
char x;
int g;

int test = 0;
 
void setup() {
  // put your setup code here, to run once:
  setup_esp_now_slave();
}

void loop() {
  // put your main code here, to run repeatedly:
  slave_send(2,test,6,7,8);
  test++;
}
