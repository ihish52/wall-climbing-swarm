 #include "esp-now_helper.h"

//extern variables in the esp-now_helper library
float allxposi[5], allyposi[5], allerro[5], allheadi[5] = {-1,-1,-1,-1,-1};
char x;
int g;
all_struct send_slave;

int test = 0;

void setup() {
  // put your setup code here, to run once:
  setup_esp_now_master();
}

void loop() {
  // put your main code here, to run repeatedly:
  master_send(test,2,3,4);

  test++;
}
