#ifndef I2CDEV_H
#define I2CDEV_H
#include "I2Cdev.h"
#endif

#ifndef WIRE_H
#define WIRE_H
#include "Wire.h"
#endif

#define FULL_SPEED 255

#define Kp  40
#define Kd  10
#define Ki  40
float heading_err = 0.0;
void updatePID();

#include <DMP_helper.h>
#include <motor_controller.h>

DMP_helper DMP;
float ypr[3];
float heading = 0.0;
float heading_ref = 0;
void print_acc();

//Motor direction offsets
const int offsetA = 1;
const int offsetB = 1;
//Motor definitions and variables
Motor motorL = Motor(AIN2, AIN1, PWMA, offsetA, STBY, PWM_CH_A);
Motor motorR = Motor(BIN1, BIN2, PWMB, offsetB, STBY, PWM_CH_B);
//Motor encoder counters - update with updateEnc();
int counterL = 0; 
int counterR = 0;
int counterAVG = 0;
int speedL = 255;
int speedR = 255;

bool fw = true;
bool turn = false;
bool stop = false;

long timer = millis();


void setup(){
	Wire.begin();
	Wire.setClock(400000);
	Serial.begin(9600);
	while (!Serial);
  
	DMP.DMP_setup();
  setupMotors();
  
}

void loop(){
  updateEnc();
  //making sure encoder ticks both count up when moving forward

  //updated motor speed based on gyro angle
  updatePID();

  counterAVG = (abs(counterR)+abs(counterL))/2.0;
  heading = ypr[0];

  if (millis() - timer > 100)
  {
    print_acc();
    Serial.println();
    Serial.print(counterL);
    Serial.print(",");
    Serial.println(counterR);
    Serial.println();
    timer = millis();
  }

  if (fw == true)
  {
    motorL.drive(speedL);
    motorR.drive(speedR);
  }

  else if (turn == true)
  {
    counterR = 0;
    counterL = 0;

    motorL.drive(FULL_SPEED);
    motorR.drive(FULL_SPEED*(-1));

    if (heading_err > -2.5 || heading_err < 2.5)
    {
      turn = false;
      fw = true;
    }
  }

  /*if (counterR > 500)
  {
    motorL.brake();
    motorR.brake();
    while(1);
  }*/

  /*while (counterL < 100 || counterR < 100)
  {
    motorL.drive(speedL);
    motorR.drive(speedR);
    updatePID();
    updateEnc();
  }*/

  if (counterAVG > 10000)
  {
    fw = false;
    turn = true;
    heading_ref += 90;
    if (heading_ref == 180)
    {
      heading_ref = -90;
    }
    counterR = 0;
    counterL = 0;
  }


}

void print_acc()
{
  DMP.ypr_pitch_bound(ypr[0], ypr[1], ypr[2]);
  Serial.print(ypr[0]);
  Serial.print("/");
  Serial.print(ypr[1]);
  Serial.print("/");
	Serial.println(ypr[2]); //Roll on X axis
}

void updatePID()
{
  //will heading error be around 5-20 degrees when moving forward ONLY?
  //e.g. Error = 10 degrees -> opposite wheel gets -10*10 = -100 speed
  heading_err = heading - heading_ref;

  //possibly the other way around
  if (heading_err < 0)
  {
    speedL = 255 - abs(Kd*heading_err);
    speedR = 255;
  }

  else if (heading_err > 0)
  {
    speedR = 255 - abs(Kd*heading_err);
    speedL = 255;
  }

  else
  {
    speedR = 255;
    speedL = 255;
  }
  
  speedL = constrain(speedL, -255, 255);
  speedR = constrain(speedR, -255, 255);
}
















