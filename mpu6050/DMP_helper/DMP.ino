#ifndef I2CDEV_H
#define I2CDEV_H
#include "I2Cdev.h"
#endif

#ifndef WIRE_H
#define WIRE_H
#include "Wire.h"
#endif

#include "DMP_helper.h"

DMP_helper DMP;
float ypr[3];

void setup(){
	Wire.begin();
	Wire.setClock(400000);
	Serial.begin(9600);
	while (!Serial);
  
	DMP.DMP_setup();
  
}

void loop(){
	DMP.ypr_pitch_bound(ypr[0], ypr[1], ypr[2]);
  Serial.print(ypr[0]);
  Serial.print("/");
  Serial.print(ypr[1]);
  Serial.print("/");
	Serial.println(ypr[2]); //Roll on X axis
}
