#ifndef I2CDEV_H
#define I2CDEV_H
#include "I2Cdev.h"
#endif

#ifndef WIRE_H
#define WIRE_H
#include "Wire.h"
#endif

#include <DMP_helper.h>
#include <motor_controller.h>
#include "BluetoothSerial.h"
#include <math.h>

//BT serial comms
BluetoothSerial ESP_BT;
char input;
//BT command strings


#define FULL_SPEED 255
#define GRID_TICK_1 10000
#define Distance_measure 100
#define Encoder_ticks 100
#define Distance_constant Distance_measure/Encoder_ticks
#define pi 3.1415926535897932384626433832795

#define Kd  10
//must be float even though heading is int
float heading_err = 0.0;
void updatePID();

DMP_helper DMP;
float ypr[3];
//used as int for switch case statements
int heading = 0;
int heading_ref = 0;
float x = 0 , y = 0;

//Motor direction offsets
const int offsetA = 1;
const int offsetB = 1;
//Motor definitions and variables
Motor motorL = Motor(AIN2, AIN1, PWMA, offsetA, STBY, PWM_CH_A);
Motor motorR = Motor(BIN1, BIN2, PWMB, offsetB, STBY, PWM_CH_B);
//Motor encoder counters - update with updateEnc();
int counterL = 0; 
int counterR = 0;
int counterAVG = 0;
int speedL = 255;
int speedR = 255;
//movement functions
void turnFixed();
void forwardFixed();
void UpdatePosition();

//booleans to decide robot motion at any given time
bool fw = false;
bool turn = false;
bool stop = false;
//boolean to show that robot is in motion - ignore remote commands
bool moving = false;
bool dir = false; //left = false, right = true


long timer = millis();

//grid variables
#define MAX_ROW 4 //5 (starts from 0)
#define MAX_COL 9 //10
#define ROBOT_NO 0
//start position for robot - left hand bottom corder or grid
int row = MAX_ROW;
int col = 0;
int loc_list[4][2] = {{row,col},{-1,-1},{-1,-1},{-1,-1}};//-1 -> not on the grid yet

//debug print functions
void print_enc();
void print_acc();


void setup(){
	Wire.begin();
	Wire.setClock(400000);
	Serial.begin(9600);
	while (!Serial);
  
  //setup gyro acc
	DMP.DMP_setup();

  //setup motors and driver
  setupMotors();

  //interrupt
  attachInterrupt(outputA1, isr_a, CHANGE);
  attachInterrupt(outputA2, isr_a, CHANGE);

  //serial comms
  ESP_BT.begin("ESP32_plsfindme"); //bluetooth device name
  
}

void isr_a()
{
  updateEnc();
  //Serial.println("updating enc");  
}

void loop(){
  //update encoder counters LR
  //updateEnc();

  //updated motor LR speed based on gyro angle
  updatePID();

  //calculate average of encoder counters LR
  counterAVG = (abs(counterR)+abs(counterL))/2.0;

  //store current heading in array from DMP
  DMP.ypr_pitch_bound(ypr[0], ypr[1], ypr[2]);
  heading = ypr[0];

  //update current list of locations fo row and col of robot
  loc_list[ROBOT_NO][0] = row;
  loc_list[ROBOT_NO][1] = col;

  //read data from BT controller
  if (ESP_BT.available())
  {
    input = ESP_BT.read();
  }
  //Serial.println(input);

  //add statements to set bool fw/turn based on BT input

  //forward - u, left - l, right - r, rev - d, nothing - x

  if (input == 'U' && moving == false)
  {
    moving = true;
    fw = true;
    Serial.println("Forward");
  }
  else if (input == 'R' && moving == false)
  {
    Serial.println("Right");
    heading_ref -= 90;
    if (heading_ref == -180) heading_ref = -90;
    else
    {
      turn = true;
      dir = false;
      moving = true;
    }
    Serial.println("Right");
  }
  else if (input == 'L' && moving == false)
  {
    Serial.println("Left");
    heading_ref += 90;
    if (heading_ref == 180) heading_ref = 90;
    else
    {
      turn = true;
      dir = false;
      moving = true;
    }
    Serial.println("Left");
  }
  /*else
  {
    motorL.brake();
    motorR.brake();
  }*/

  //MUST UPDATE HERE AGAIN
  //updated motor LR speed based on gyro angle
  updatePID();

  //movement logic based on boolean states
  if (fw == true)
  {
    forwardFixed();
  }

  else if (turn == true)
  {
    turnFixed();
  }
  else
  {
    motorL.brake();
    motorR.brake();

  }

  //1 second print timer
  if (millis() - timer > 1000)
  {
    print_acc();
    print_enc();
    timer = millis();
    Serial.println(heading_ref);
  }

}

void turnFixed()
{
  counterR = 0;
  counterL = 0;

  if (dir == false)
  {
    motorR.drive(FULL_SPEED*(-1));
    motorL.drive(FULL_SPEED);
  }
  else if (dir == true)
  {
    motorL.drive(FULL_SPEED);
    motorR.drive(FULL_SPEED*(-1));
  }
  
  //condition to stop turning
  if (abs(heading_err) < 5)
  {
    Serial.println("turn stop ");
    Serial.print(abs(heading_err));
    turn = false;
    counterR = 0;
    counterL = 0;

    moving = false;
  }
}

void forwardFixed()
{
  //grid tracking 
  /*
  if (counterL == 0 || counterR == 0)
  {
    switch (heading_ref)
    {
      case 0://moving up
        if (row > 0) row -= 1;
        else stop = true;
        break;
      case 90://moving right
        if (col < MAX_COL) col += 1;
        else stop = true;
        break;
      case -90://moving left
        if (col > 0) col -= 1;
        else stop = true;
        break;
      //add backwards reverse case
    }
  }
  */
  if(1)
  //if (stop == false)
  {
    motorL.drive(speedL);
    motorR.drive(speedR);
  }

  //condition to stop forward motion - after travelling fixed ticks
  if (counterAVG > GRID_TICK_1 || (counterAVG == 0 && stop == true))
  {
    Serial.println("Stopping forward");
    fw = false;
    stop = false;
    counterR = 0;
    counterL = 0;

    moving = false;
  }
}

void UpdatePosition(){
  float distance_moved = counterAVG*Distance_constant;
  x = distance_moved *cos(90 - ((180/pi)*heading));
  y = distance_moved * sin(90-((180/pi)*heading));
  counterL=0;
  counterR=0;
  
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

//proportional control for moving straight
void updatePID()
{
  //will heading error be around 5-20 degrees when moving forward ONLY?
  //e.g. Error = 10 degrees -> opposite wheel gets -10*10 = -100 speed
  //Serial.print("Heading error:");
  heading_err = heading - heading_ref;
  //Serial.println(heading_err);

  //possibly the other way around
  if (heading_err < 0)
  {
    speedL = 255 - abs(Kd*heading_err);
    speedR = 255;
  }

  else if (heading_err > 0)
  {
    speedR = 255 - abs(Kd*heading_err);
    speedL = 255;
  }

  else
  {
    speedR = 255;
    speedL = 255;
  }
  
  speedL = constrain(speedL, -255, 255);
  speedR = constrain(speedR, -255, 255);
}


