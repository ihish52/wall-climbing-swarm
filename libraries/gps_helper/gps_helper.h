#include <Arduino.h>
#include <Adafruit_GPS.h>

#define GPSSerial Serial2
#define LEDPIN 2

#define GPSECHO false

#define LAT_FIXED 50.937439505000036 //50.937439505000036, -1.3963454982796897
#define LON_FIXED -1.3963454982796897
//float paceArray[10];

//extern Adafruit_GPS GPS(&GPSSerial);

void setupGPS();
void echoCheckGPS();
float conv_coords(float in_coords);
float getLat();
float getLon();
double haversine(double lat1, double lon1, double lat2, double lon2);
float distanceToFixedLoc();
int GPSFIX();
float speedGPS();
//new in arm warmer v5.1 because sleep/wake requires serial flushes
void flushGPStx();
void flushGPSrx();
void GPSstandby();
void GPSwake();