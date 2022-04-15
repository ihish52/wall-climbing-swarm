 #include <esp-now_helper.h>

//extern variables in the esp-now_helper library
float allxposi[5], allyposi[5], allerro[5], allheadi[5];
char x;
//int d;

long print_timer = millis();
#define PRINT_TIME 1000

void setup() {
  // put your setup code here, to run once:
  setup_esp_now_controller();
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  controller_send();

  if (millis() - print_timer > PRINT_TIME)
  {
    Serial.print(allxposi[0]);
    Serial.print(',');
    Serial.println(allyposi[0]);
    Serial.println(allheadi[0]); 
    Serial.println(x);
    //Serial.println(d);
    
    print_timer = millis();
  }

}
