//Motor controller library for wall climbers combining SparkFun_TB6612 and incremental encoders

#include <SparkFun_TB6612.h>

// Pins for all inputs, keep in mind the PWM defines must be on PWM pins
// the default pins listed are the ones used on the Redbot (ROB-12097) with
// the exception of STBY which the Redbot controls with a physical switch
#define AIN1 2
#define BIN1 7
#define AIN2 4
#define BIN2 8
#define PWMA 5
#define PWMB 6
#define STBY 9

//PWM channels required for ESP32
#define PWM_CH_A 0
#define PWM_CH_B 1

//Motor encoder outputs
//A1 B1 - Motor 1
//A2 B2 - Motor 2
#define outputA1 62
#define outputB1 72
#define outputA2 63
#define outputB2 73
 


void setupMotors();
//must run in every loop
void updateEnc();


