#include "compass_helper.h"

extern LSM303 compass;
extern int heading_offset;

void setupCompass(){
	compass.init();
	compass.enableDefault();
	compass.m_min = (LSM303::vector<int16_t>){-DEFAULT_MIN, -DEFAULT_MIN, -DEFAULT_MIN};
	compass.m_max = (LSM303::vector<int16_t>){DEFAULT_MAX, DEFAULT_MAX, DEFAULT_MAX};	
}
void calibrate_min(int x, int y, int z){
	compass.m_min = (LSM303::vector<int16_t>){x, y, z};
}
void calibrate_max(int x, int y, int z){
	compass.m_max = (LSM303::vector<int16_t>){x, y, z};
}

float getHeading(){
  compass.read();
  float heading = compass.heading() - heading_offset;
  if(heading < 0) heading += 360;
  //make the compass turn in the opposite clockwise direction
  return 360 - heading;
}

float guidingHeading(){
  compass.read();
  float heading = compass.heading() - heading_offset;
  if(heading < 0) heading += 360;
  //make the compass turn in the opposite clockwise direction
  return 360 - heading;
}