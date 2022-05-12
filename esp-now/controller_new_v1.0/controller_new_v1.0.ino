 #include <esp-now_helper.h>

//extern variables in the esp-now_helper library
float allxposi[5], allyposi[5], allerro[5], allheadi[5];
char x;
int g;
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


    Serial.println("Master (0):");
    Serial.print("xy: ");
    Serial.print(allxposi[0]);
    Serial.print(" , ");
    Serial.println(allyposi[0]);
    Serial.print("heading: ");
    Serial.println(allheadi[0]);
    Serial.print("cmd: ");
    Serial.println(x);
    Serial.print("err: ");
    Serial.println(allerro[0]);

    Serial.println();
    Serial.println();

    /*Serial.println("Slave (1):");
    Serial.print("xy: ");
    Serial.print(allxposi[1]);
    Serial.print(" , ");
    Serial.println(allyposi[1]);
    Serial.print("heading: ");
    Serial.println(allheadi[1]);
    Serial.print("cmd: ");
    Serial.println(x);
    Serial.print("err: ");
    Serial.println(allerro[1]);

    Serial.println();
    Serial.println();*/

    Serial.println("Slave (2):");
    Serial.print("xy: ");
    Serial.print(allxposi[2]);
    Serial.print(" , ");
    Serial.println(allyposi[2]);
    Serial.print("heading: ");
    Serial.println(allheadi[2]);
    Serial.print("cmd: ");
    Serial.println(x);
    Serial.print("err: ");
    Serial.println(allerro[2]);

    Serial.println();
    Serial.println();

    /*Serial.println("Slave (3):");
    Serial.print("xy: ");
    Serial.print(allxposi[3]);
    Serial.print(" , ");
    Serial.println(allyposi[3]);
    Serial.print("heading: ");
    Serial.println(allheadi[3]);
    Serial.print("cmd: ");
    Serial.println(x);
    Serial.print("err: ");
    Serial.println(allerro[3]);

    Serial.println();
    Serial.println();*/

    Serial.println("Slave (4):");
    Serial.print("xy: ");
    Serial.print(allxposi[4]);
    Serial.print(" , ");
    Serial.println(allyposi[4]);
    Serial.print("heading: ");
    Serial.println(allheadi[4]);
    Serial.print("cmd: ");
    Serial.println(x);
    Serial.print("err: ");
    Serial.println(g); //allerro[4]

    Serial.println();
    Serial.println();

    print_timer = millis();
  }

}
