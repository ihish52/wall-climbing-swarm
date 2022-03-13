#ifndef I2CDEV_H
#define I2CDEV_H
#include "I2Cdev.h"
#endif

#ifndef WIRE_H
#define WIRE_H
#include "Wire.h"
#endif

#define Kp  40
#define Kd  10
#define Ki  40
float heading_err = 0.0;
void updatePID();

#include "DMP_helper.h"
#include <motor_controller.h>

DMP_helper DMP;
float ypr[3];
float heading = 0.0;
float heading_ref = 0;

//Motor direction offsets
const int offsetA = 1;
const int offsetB = -1;
//Motor definitions and variables
Motor motorL = Motor(AIN1, AIN2, PWMA, offsetA, STBY, PWM_CH_A);
Motor motorR = Motor(BIN1, BIN2, PWMB, offsetB, STBY, PWM_CH_B);
//Motor encoder counters - update with updateEnc();
int counterL = 0; 
int counterR = 0;
int speedL = 255;
int speedR = 255;


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

  //updated motor speed based on gyro angle
  updatePID();
  
	DMP.ypr_pitch_bound(ypr[0], ypr[1], ypr[2]);
  Serial.print(ypr[0]);
  Serial.print("/");
  Serial.print(ypr[1]);
  Serial.print("/");
	Serial.println(ypr[2]); //Roll on X axis

  heading = ypr[2];
}

void updatePID()
{
  //will heading error be around 5-20 degrees when moving forward ONLY?
  //e.g. Error = 10 degrees -> opposite wheel gets -10*10 = -100 speed
  heading_err = Kd*(heading - heading_ref);

  //possibly the other way around
  if (heading_err > 0)
  {
    speedL = 255 - heading_err;
    speedR = 255;
  }

  else if (heading_err < 0)
  {
    speedR = 255 - heading_err;
    speedL = 255;
  }
  
  speedL = constrain(speedL, -255, 255);
  speedR = constrain(speedR, -255, 255);
}
















