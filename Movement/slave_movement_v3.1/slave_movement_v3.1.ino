// code for the master
#ifndef I2CDEV_H
#define I2CDEV_H
#include "I2Cdev.h"
#endif

#ifndef WIRE_H
#define WIRE_H
#include "Wire.h"
#endif

#define ID 1

#define KP 7
#define KD 0.001

#define FULL_SPEED 255
#define Distance_measure 36.25
#define Encoder_ticks 10000
#define Distance_constant Distance_measure/Encoder_ticks
#define MATH_PI 3.141592653589793

#define MAX_ROBOT_NUM 4
#define calibrate_ticks 5000

#define PRINT_TIME 1000

#define POSITION_UPDATE 20

#define FTL_TICKS 50

#include <esp-now_helper.h>
#include <DMP_helper.h>
#include <motor_controller.h>

//FTL variables
bool FTL = true;

//Clustering variables
bool cluster = false; //default off
float m_s_ypos = 0;
float m_s_xpos = 0;
float m_s_heading = 0;

char input,previnput;

//extern variables in the esp-now_helper library
float allxposi[5], allyposi[5], allerro[5], allheadi[5];
char x;
int g;

long position_timer = millis();
long print_timer = millis();

//calibration variables
bool calibrate = true;
float positions[4][4] = {{0,0,0,0},{-1,-1,-1,-1},{-1,-1,-1,-1},{-1,-1,-1,-1}};
uint32_t calibrate_timer = millis();
int test_counter = 0;

//DMP and heading error variables
DMP_helper DMP;
float ypr[3];
float heading = 0;
float xpos = 0 , ypos = 0;
float heading_ref = 0;
float heading_error, heading_error_prev = 0;

//Motor direction offsets
const int offsetA = 1;
const int offsetB = 1;

//Motor definitions and variables
float Kp = KP;//5;
float Kd = KD;//0.0008;
Motor motorL = Motor(AIN2, AIN1, PWMA, offsetA, STBY, PWM_CH_A);
Motor motorR = Motor(BIN1, BIN2, PWMB, offsetB, STBY, PWM_CH_B);

//Encoder counting variables
int counterL = 0; 
int counterR = 0;
int counterAVG = 0;

//driveL/R {-255,255} //speedL/R {-1,1}
int driveL, driveR = 0;
int speedL, speedR = 0;
float controlL, controlR = 0;
uint32_t timer_PID, timer_PID_prev , elapsed_timer_PID = 0;

//movement functions
void forward();
void turn();
void reverse();
void stopp();

//Position Function
void UpdatePosition();

//debug print functions
void print_enc();
void print_acc();

//robot states
enum state{
  FORWARD, REVERSE, TURN, STOP
};
state current_state=STOP;


void setup() {
  
  Wire.begin();
  Wire.setClock(400000);
  Serial.begin(9600); //9600

  //setup gyro acc
  DMP.DMP_setup();

  //setup motors and driver
  setupMotors();

  //setup ESP-NOW
  //setup_esp_now_master();
  setup_esp_now_slave(ID, 0, heading, xpos, ypos);

  //attach same ISR to encoder outputs
  attachInterrupt(outputA1, isr_a, CHANGE);
  attachInterrupt(outputA2, isr_a, CHANGE);


}

//ISR to update encoder counters
void isr_a()
{
  updateEnc();
}

void loop() {
  
  input = x;
  
  if (millis() - print_timer > PRINT_TIME)
  {
    Serial.print("input:");
    Serial.println(x);
    Serial.print("xpos: ");
    Serial.print(xpos); //xpos
    Serial.print(" ypos: ");
    Serial.print(ypos);

    print_enc();

    print_timer = millis();
  }
  
  master_send(0,heading,xpos,ypos); 

  //calculate average of encoder counters LR
  counterAVG = ((counterR)-(counterL))/2.0;

  //store current heading in array from DMP
  DMP.ypr_pitch_bound(ypr[0], ypr[1], ypr[2]);
  heading = ypr[2];
  heading_error = heading - heading_ref;
  
  //PD controller
  timer_PID_prev = timer_PID;
  timer_PID = millis();
  elapsed_timer_PID = timer_PID - timer_PID_prev;

  //Follow the leader mode - on by default after calibration
  if(input == 'FTL' && FTL == false) FTL = true; // press FTL once to turn on
  if (input == 'FTL' && FTL == true) FTL = false; // press FTL twice to turn off

  //Clustering control - press clustering button to toggle clustering on and off
  if(input == 'clustering' && cluster == false) cluster = true; 
  if (input == 'clustering' && cluster == true) cluster = false; 

  //calibrate code deactivated because calibrate bool initialized to false
  if (calibrate == true)
  {
    if (millis() - calibrate_timer > 3000){
      //check y position of each robot
      //initialize slave y positions with -1
      if (allyposi[0] == 0 || allyposi[1] == 0 || allyposi[2] == 0 || allyposi[3] == 0) 
      {
        current_state=FORWARD;
        forward();

      }
      if (counterAVG > calibrate_ticks){
        current_state=STOP;
        stopp();
        counterL = 0;
        counterR = 0;
        counterAVG = 0;

        if (allyposi[3] != -1) calibrate = false;

        positions[test_counter+1][1] = 0;
        test_counter++;   
      }
      
    }
  }

  //MASTER FTL code
  else if (FTL == true)
  {
    //master_send() above already sends command character from remote
    //slave uses these as controller commands
      if(input == 'U'){
        current_state=FORWARD;
        if(previnput !='U'){
        }
        forward();
      }
      else if (input == 'D'){
        current_state=REVERSE;
        if(previnput !='D'){
        }
        reverse();
      }
      else if((input == 'L')&&(previnput!='L')){
        current_state = TURN;
        heading_ref -= 15;
        if(heading_ref < -179)
          heading_ref = 180;
      }
      else if((input == 'R')&&(previnput!='R')){
        current_state = TURN;
        heading_ref += 15;
        if(heading_ref > 180)
          heading_ref = -179;  
      }
      else{
        current_state=STOP;
          stopp();
      }

      previnput=input;
  }

  else if (cluster == true)
  {
    //calculate slave's heading to master's x,y coordinates
    m_s_ypos = allyposi[0] - allyposi[ID];
    m_s_xpos = allxposi[0] - allxposi[ID];

    //calculate slave's heading from 0 to the master
    m_s_heading = atan2(m_s_xpos/m_s_ypos);

    
  }

  else
  {
    current_state=STOP;
    stopp();
  }

  turn();

  driveL = constrain((200*speedL + controlL), -255, 255);
  driveR = constrain((-200*speedR + controlR), -255, 255);
  
  motorL.drive(driveL);
  motorR.drive(driveR);
  
  heading_error_prev = heading_error;
  
  if (millis() - position_timer > POSITION_UPDATE)
  {
    UpdatePosition();
    position_timer = millis();
  }
  
  //Serial.println(heading_ref);


}

//reset encoders after turning without updating position
void ResetEnc()
{
  counterL=0;
  counterR=0;
  counterAVG=0;
}

void UpdatePosition(){
  float distance_moved = counterAVG*Distance_constant;
  ypos += distance_moved *cos((MATH_PI/180)*(heading));
  xpos += distance_moved * sin((MATH_PI/180)*heading);

  //uncomment again when debugging is done
  counterL=0;
  counterR=0;
  counterAVG=0;

  //prints performed in print timer in loop
  //Serial.print("X = ");
  //Serial.println(x);
  //Serial.print("Y = ");
  //Serial.println(y);
  //positions[datarec.id][0] = x;
  //positions[datarec.id][1] = y;
}

void forward(){
	speedL = 1;
	speedR = -1;
}
void reverse(){
	speedL = -1;
	speedR = 1;
}
void stopp(){
	speedL = 0;
	speedR = 0;
}

void turn(){

  if(abs(heading-heading_ref) > 180){
    controlL = Kp*heading_error + Kd*(heading_error - heading_error_prev);
	  controlR = -controlL;
  }
  else{
    controlR = Kp*heading_error + Kd*(heading_error - heading_error_prev);
    controlL = -controlR;
  }
}

//printer encoder counter values
void print_enc()
{
  Serial.println();
  Serial.print(counterL);
  Serial.print(",");
  Serial.println(counterR);
  Serial.println();
}

//print yaw pitch roll values from DMP
void print_acc()
{
  //DMP.ypr_pitch_bound(ypr[0], ypr[1], ypr[2]);
  Serial.print(ypr[0]);
  Serial.print("/");
  Serial.print(ypr[1]);
  Serial.print("/");
  Serial.println(ypr[2]); //Roll on X axis
}
