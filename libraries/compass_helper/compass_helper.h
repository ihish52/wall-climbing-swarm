#include <Wire.h>
#include <LSM303.h>

#define DEFAULT_MAX 32767
#define DEFAULT_MIN -32768

void setupCompass();
void calibrate_min(int x, int y, int z);
void calibrate_max(int x, int y, int z)	;
float getHeading();
float guidingHeading();
