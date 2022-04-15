 #include <esp-now_helper.h>

 //extern variables in the esp-now_helper library
float allxposi[5], allyposi[5], allerro[5], allheadi[5];
char x;
int g; //can delete after test

void setup() {
  // put your setup code here, to run once:
  setup_esp_now_master();
}

void loop() {
  // put your main code here, to run repeatedly:
  master_send(1,2,3,4);
}
