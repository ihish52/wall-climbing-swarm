#include "eeprom_helper.h"
#include "gps_helper.h"

//track completed/passed waypoints on the route with bool true/false
bool waypointTracker[NUM_COORDS] = {false};
extern uint8_t progress = 0;
extern uint8_t eepromAddr_i;

//eeprom setup at start of run
void setupEEPROM()
{
  EEPROM.begin(EEPROM_SIZE);
  //set all waypoints to false to begin run
  for (uint16_t i=0; i<NUM_COORDS; i++) waypointTracker[i] = false;
}

//checks progress against a lat/lon GPS reading and updates  waypointTracker
//assumes that EEPROM addresses start from 0
uint8_t checkProgress(float gpsLat, float gpsLon)
{
  uint16_t eepromAddr = EEPROM_START_ADDR; //byte address of the eeprom address
  uint8_t currentProgress = 0;

  //checking distance within this function by printing
  //Serial.println(haversine(getLat(), getLon(), EEPROM.readFloat(eepromAddr), EEPROM.readFloat(eepromAddr+4)));
  /*Serial.println(getLon());
  Serial.println(getLat());
  Serial.println(EEPROM.readFloat(eepromAddr));
  Serial.println(EEPROM.readFloat(eepromAddr+4));*/

  //Serial.println((uint8_t)((((0/8.0)+1)/1)*100.0));

  for (uint16_t i=0; i<NUM_COORDS; i++)
  {
    //reads assuming EEPROM has float lon,lat pairs of readings - no altitude
    //float eepromLat = EEPROM.readFloat(eepromAddr); -- directly in haversine function below
    //float eepromLon = EEPROM.readFloat(eepromAddr+4); -- directly in haversine function below
    
    //checking if distance from eeprom coord to gps coord is < WAYPOINT_RADIUS
    if (haversine(getLat(), getLon(), EEPROM.readFloat(eepromAddr+4), EEPROM.readFloat(eepromAddr)) < WAYPOINT_RADIUS)
    {
        waypointTracker[i] = true;
        //current progress = (current reading number / number of coords) * 100 % --> rounded down by casting to uint8_t
        //CHECK IF THIS ROUNDING WORKS!!!!! 
        currentProgress = (uint8_t)(((eepromAddr/8.0)/(NUM_COORDS-1))*100.0);

		    eepromAddr_i = (uint8_t)eepromAddr/8;
        //updating progress if currentProgress is greater
        if (currentProgress > progress) 
		      {
			      progress = currentProgress;
		      }

    }

    eepromAddr += 8;
  }

  return progress;
}
