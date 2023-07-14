#include <motor_controller.h>

//Motor direction offsets
const int offsetA = 1;
const int offsetB = -1;
//Motor definitions and variables
Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY, PWM_CH_A);
Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, STBY, PWM_CH_B);
//Motor encoder counters - update with updateEnc();
int counterL = 0; 
int counterR = 0;

int speed = 255;
long ticktimer = millis();


void setup() {
  // put your setup code here, to run once:
  setupMotors();
  Serial.begin(9600);
  Serial.println("test");
}

void loop() {
  //update encoder counters in every loop - counter1/2 variables updated
  updateEnc();

  if (millis()-ticktimer > 1000)
  {
    Serial.print(counterL);
    Serial.print(",");
    Serial.println(counterR);
    ticktimer = millis();
    //speed = speed * (-1);
  }

  //Serial.println("loop");
  
  // example motor controls from modified SparkFun_TB6612 library
   motor1.drive(speed);
   motor2.drive(speed);

   //motor1.drive(-255,1000);
   //motor1.brake();
   //delay(1000);
   
   //motor2.drive(255,1000);
   //motor2.drive(-255,1000);
   //motor2.brake();  
   //delay(1000);

}
