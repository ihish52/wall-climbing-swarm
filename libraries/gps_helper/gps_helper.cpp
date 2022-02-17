#include "gps_helper.h"

//extern Adafruit_GPS GPS(&GPSSerial);
Adafruit_GPS GPS(&GPSSerial);

void GPSstandby()
{
	GPS.standby();
}

void GPSwake()
{
	GPS.wakeup();
}

//new because deep sleep mode requires serial flushes
void flushGPStx()
{
	GPSSerial.flush();
}

void flushGPSrx()
{
	while(GPSSerial.available()) GPSSerial.read();
	GPS.standby();
	GPSSerial.end();
}

void setupGPS()
{
	GPS.begin(9600);
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
    GPS.sendCommand(PGCMD_ANTENNA);
    delay(1000);
    GPSSerial.println(PMTK_Q_RELEASE);
	
	//necessary to flush out old serial data after wake from deep sleep
	while(GPSSerial.available()) GPSSerial.read();

}

void echoCheckGPS()
{
	char c = GPS.read();
	if (GPSECHO)
		if (c) Serial.print(c);
	if (GPS.newNMEAreceived()) {
		//Serial.print(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
		if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
		return; // we can fail to parse a sentence in which case we should just wait for another
  }
}

float conv_coords(float in_coords)
 {
 //Initialize the location.
 float f = in_coords;
 // Get the first two digits by turning f into an integer, 
 // then doing an integer divide by 100;
 // firsttowdigits should be 77 at this point.
 int firsttwodigits = ((int)f)/100; //This assumes that f < 10000.
 float nexttwodigits = f - (float)(firsttwodigits*100);
 float theFinalAnswer = (float)(firsttwodigits + nexttwodigits/60.0);
 return theFinalAnswer;
 }
 
 double haversine(double lat1, double lon1, double lat2, double lon2) {
    const double rEarth = 6371000.0; // in meters
    double x = pow( sin( ((lat2 - lat1)*M_PI/180.0) / 2.0), 2.0 );
    double y = cos(lat1*M_PI/180.0) * cos(lat2*M_PI/180.0);
    double z = pow( sin( ((lon2 - lon1)*M_PI/180.0) / 2.0), 2.0 );
    double a = x + y * z;
    double c = 2.0 * atan2(sqrt(a), sqrt(1.0-a));
    double d = rEarth * c;
    // Serial.printlnf("%12.9f, %12.9f, %12.9f, %12.9f, %12.9f, %12.9f", x, y, z, a, c, d);
    return d; // in meters
}

//get decimal latitude from GPS module
float getLat()
{
	float latitude = conv_coords(GPS.latitude);
    if (GPS.lat == 'S') latitude *= -1;
	return latitude;
}

//get decimal longitude from GPS module
float getLon()
{
	float longitude = conv_coords(GPS.longitude);
    if (GPS.lon == 'W') longitude *= -1;
	return longitude;
}

float distanceToFixedLoc()
{
	return haversine(getLat(), getLon(), LAT_FIXED, LON_FIXED);
}

int GPSFIX()
{
  if (GPS.fix) return 1;
  else return 0;
}

//returns speed in KNOTS -- needs conversion
float speedGPS()
{
	return GPS.speed;
}

/*//Standalone function to calculate moving average of 'ADC_AVG_NUM' readings
uint16_t movingAvg(uint16_t (*adcAvgArr)[ADC_AVG_NUM], long (*adcSums),
                   uint8_t pos, uint16_t adcReading, uint8_t index)
{
  //Subtract the oldest number from the prev sum, add the new number
  *(adcSums + index) = *(adcSums + index) - *(*(adcAvgArr + index) + pos) + adcReading;
  //Assign the nextNum to the position in the array
  *(*(adcAvgArr + index) + pos) = adcReading;
  //return the average
  return *(adcSums + index) / ADC_AVG_NUM;
}*/