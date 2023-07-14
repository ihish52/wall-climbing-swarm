// code for slave

#include <math.h>

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
#define KT 0.05

#define FULL_SPEED 255
#define Distance_measure 36.25
#define Encoder_ticks 10000
#define Distance_constant Distance_measure/Encoder_ticks
#define MATH_PI 3.141592653589793

#define MAX_ROBOT_NUM 3
#define CALIBRATE_DIST 25

#define PRINT_TIME 1000

#define POSITION_UPDATE 20

#define FIF_TICKS 50
#define BUTTON_COOLDOWN 3000

#define FTL_STOP_DIST 15

#define CLUSTER_CLR 20

#include <esp-now_helper.h>
#include <DMP_helper.h>
#include <motor_controller.h>

//encoder ticks correction gain
float Kt = KT;

//FTL variables
bool FTL = false;
float slave_dist[MAX_ROBOT_NUM - 1] = {CALIBRATE_DIST, CALIBRATE_DIST};
void getSlaveDist();

//FIF variables
bool FIF = true;
long button_mode_timer = millis();

//Clustering variables
bool cluster = false; //default off
float m_s_ypos = 0;
float m_s_xpos = 0;
float m_s_dist = 0;
float m_s_heading = 0;

char input,previnput;

//extern variables in the esp-now_helper library
float allxposi[5], allyposi[5], allerro[5], allheadi[5] = {-1,-1,-1,-1,-1};
char x;
int g;

long position_timer = millis();
long print_timer = millis();

//calibration variables
bool calibrate = true;
float positions[4][4] = {{0,0,0,0},{-1,-1,-1,-1},{-1,-1,-1,-1},{-1,-1,-1,-1}};
uint32_t calibrate_timer = millis();
int test_counter = 1;

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
  setup_esp_now_slave();

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
  
  slave_send(ID, m_s_dist, heading, xpos, ypos);//(ID, 9,10,11,12);

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

    //Follow in formation mode - on by default after calibration
    if(input == 'T' && FIF == false && previnput != 'T') FIF = true; // press FIF once to turn on
    else if (input == 'T' && FIF == true  && previnput != 'T') FIF = false; // press FIF twice to turn off

    //Follow in formation mode - on by default after calibration
    //if(input == 'T' && FTL == false && previnput != 'T') FTL = true; // press FTL once to turn on
    //else if (input == 'T' && FTL == true  && previnput != 'T') FTL = false; // press FTL twice to turn off

    //Clustering control - press clustering button to toggle clustering on and off
    if(input == 'S' && cluster == false  && previnput != 'S') cluster = true; 
    else if (input == 'S' && cluster == true  && previnput != 'S') cluster = false;


  //calibrate code deactivated because calibrate bool initialized to false
  if (calibrate == true)
  {
    if (millis() - calibrate_timer > 3000){
      //check y position of each robot
      //initialize slave y positions with -1
      if (allyposi[0] == -1 || allyposi[MAX_ROBOT_NUM-2] == 0 || allyposi[MAX_ROBOT_NUM-1] == 0)// || allyposi[2] == 0 || allyposi[3] == 0) 
      {
        //Serial.println("HERE");
        //Serial.println(allyposi[0]);
        //Serial.println(allyposi[1]);
        
        current_state=FORWARD;
        forward();

      }
      if (ypos > CALIBRATE_DIST*test_counter){
        current_state=STOP;
        stopp();
        ResetEnc();

        if (allyposi[MAX_ROBOT_NUM-1] != 0 && allyposi[MAX_ROBOT_NUM-1] != -1) calibrate = false;

        //positions[test_counter+1][1] = 0;
        test_counter++;   
      }
      
    }
  }

  //MASTER FIF code
  else if (FIF == true)
  {
    //Serial.println("FOLLOW MODE");
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

      
  }

  else if (FTL == true)
  {
    m_s_ypos = allyposi[0] - allyposi[ID];
    m_s_xpos = allxposi[0] - allxposi[ID];

    m_s_dist = sqrt(pow(m_s_xpos,2)+pow(m_s_ypos,2));

    //calculate slave's heading from 0 to the master
    m_s_heading = (180/MATH_PI)*atan2(m_s_xpos,m_s_ypos);

    heading_ref = m_s_heading;

    if (slave_dist[0] <= FTL_STOP_DIST || slave_dist[MAX_ROBOT_NUM-2] <= FTL_STOP_DIST)
    //if (m_s_dist<= FTL_STOP_DIST)
    {
      current_state=STOP;
      stopp();
    }
    else
    {
      current_state=FORWARD;
      forward();
    }

  }

  else if (cluster == true)
  {
    //Serial.println("CLUSTER MODE");
    //calculate slave's heading to master's x,y coordinates
    m_s_ypos = allyposi[0] - allyposi[ID];
    m_s_xpos = allxposi[0] - allxposi[ID];

    m_s_dist = sqrt(pow(m_s_xpos,2)+pow(m_s_ypos,2));

    //calculate slave's heading from 0 to the master
    m_s_heading = (180/MATH_PI)*atan2(m_s_xpos,m_s_ypos);

    heading_ref = m_s_heading;

    //Serial.println(heading_ref);

    if (slave_dist[0] <= FTL_STOP_DIST || slave_dist[MAX_ROBOT_NUM-2] <= FTL_STOP_DIST)
    {
      current_state=STOP;
      stopp();
    }
    else
    {
      current_state=FORWARD;
      forward();
    }
  }

  else
  {
    current_state=STOP;
    stopp();
  }

  previnput=input;

  getSlaveDist();

  turn();

  driveL = constrain((200*speedL + controlL), -255, 255);
  driveR = constrain((-200*speedR + controlR), -255, 255);

  if (millis() - position_timer > POSITION_UPDATE)
  {
    UpdatePosition();
    position_timer = millis();
  }

  //if (abs(counterL) > abs(counterR)) driveL = constrain(driveL - ((abs(counterL) - abs(counterR))*Kt), -255, 255);
  //else if (abs(counterR) > abs(counterL)) driveR = constrain(driveR - ((abs(counterR) - abs(counterL))*Kt), -255, 255);
  
  motorL.drive(driveL);
  motorR.drive(driveR);
  
  heading_error_prev = heading_error;
  
  //Serial.println(heading_ref);


}

//get distance of current slave to all other slaves and master
void getSlaveDist()
{

  for (int i = 0; i < MAX_ROBOT_NUM; i++)
  {
    if (i == ID) continue;

    float y_dist = allyposi[i] - allyposi[ID];
    float x_dist = allxposi[i] - allxposi[ID];

    float xy_dist = sqrt(pow(x_dist,2)+pow(y_dist,2));

    slave_dist[i] = xy_dist;

  }

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
