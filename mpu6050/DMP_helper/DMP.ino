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

/*
counter1 and counter2 generate values from the motor encoders
#define FULL_TURN 380    // Example number which the counter1 or counter 2 increments when the motor makes a full 360 degree turn
#define WHEEL_DIAMETER 60	//(mm) including the extruding magnets, this will be used for calculating the circumference of the wheel

volatile float travelled_distance[2] = {0}; // a distance for both wheels

For now i'll assume theres only clockwise and anticlockwise spin (at max rate)

Forward traveling in meters:
			v 188.5mm v
	Distance = pi*WHEEL_DAMETER * counter/FULL_TURN;

THEREFORE :) a meter travel will be
	if(Distance<1000)
		forward_L();
		forward_R();
	else
		stop_L();
		stop_R();
	
	LMAO

Now the center SPIN:

#define ROBOT_DIAMETER 200 // (mm)
					v 628.32 mm v
	Circumference of robot would be pi*200

	90 degrees turn clockwise would simply be:   /4 because a quarter of the circumference translates to 90 degrees of a spin
	if(distance < pi*200/4)
		forward_L();
		backward_R();
	else
		stop_L();
		stop_R();

*/



