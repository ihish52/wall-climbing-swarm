#include <EEPROM.h>

#define EEPROM_SIZE 4096 //4kb for ESP32
#define EEPROM_START_ADDR 0 //Address that the code sequentially starts checking waypoints from
#define NUM_COORDS 8 //number of reading pairs in eeprom
#define WAYPOINT_RADIUS 100 //in metres


//eeprom setup at start of run
void setupEEPROM();
//checks progress, updates  waypointTracker. outputs percentage completion
//must be run every x seconds of the run
uint8_t checkProgress(float lat, float lon);