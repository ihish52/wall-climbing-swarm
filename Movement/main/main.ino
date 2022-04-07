#ifndef I2CDEV_H
#define I2CDEV_H
#include "I2Cdev.h"
#endif

#ifndef WIRE_H
#define WIRE_H
#include "Wire.h"
#endif
#ifndef DMP_HELPER_H
#define DMP_HELPER_H
#include "DMP_helper.h"
#endif
#include "motor_controller.h"
#include "BluetoothSerial.h"

//BT serial comms
BluetoothSerial ESP_BT;
char input;
//BT command strings


#define FULL_SPEED 255
#define Kd  10
#define Distance_measure 100
#define Encoder_ticks 100
#define Distance_constant Distance_measure/Encoder_ticks
#define pi 3.1415926535897932384626433832795

float heading_err = 0.0;
DMP_helper DMP;
float ypr[3];
float heading = 0;
float x = 0 , y = 0;
float heading_ref = 0;

//Motor direction offsets
const int offsetA = 1;
const int offsetB = 1;
//Motor definitions and variables
Motor motorL = Motor(AIN2, AIN1, PWMA, offsetA, STBY, PWM_CH_A);
Motor motorR = Motor(BIN1, BIN2, PWMB, offsetB, STBY, PWM_CH_B);
//Encoders
int counterL = 0; 
int counterR = 0;
int counterAVG = 0;
int speedL = 255;
int speedR = 255;

//movement functions
void turn(float heading_refference);
void forward(float heading_refference);
void reverse(float heading_refference);
void stopp(float heading_refference);

//Position Function
void UpdatePosition();

long timer = millis();

//debug print functions
void print_enc();
void print_acc();

enum state{
  FORWARD, REVERSE, TURN, STOP
};

state current_state=STOP;

void setup() {
  
  Wire.begin();
  Wire.setClock(400000);
  Serial.begin(9600);
  //Serial.println("START");
  //while (!Serial);

  //setup gyro acc
  DMP.DMP_setup();
  Serial.println("START2");

  //setup motors and driver
  setupMotors();
  Serial.println("START3");

  //serial comms
  ESP_BT.begin("ESP32_plsfindme"); //bluetooth device name

  Serial.println("START4");


}

void loop() {
  //Serial.println("START");
  print_acc();
  if ((current_state == FORWARD)||(current_state == REVERSE)){
    UpdatePosition();
  }
  //update encoder counters LR
  updateEnc();

  //update position
  


  //calculate average of encoder counters LR
  counterAVG = (abs(counterR)+abs(counterL))/2.0;
   //read data from BT controller
  if (ESP_BT.available())
  {
    input = ESP_BT.read();
  }
  //Serial.println(input);

  //store current heading in array from DMP
  DMP.ypr_pitch_bound(ypr[0], ypr[1], ypr[2]);
  heading = ypr[0];
  if(input=='U'){
    current_state=FORWARD;
    forward(heading_ref);
  }
  else if (input == 'D'){
    current_state=REVERSE;
    reverse(heading_ref);
  }
  else if(input == 'L'){
    current_state = TURN;
    turn(heading_ref);
    heading_ref+=1;
    counterL=0;
    counterR=0;
  }
  else if(input == 'R'){
    current_state = TURN;
    turn(heading_ref);
    heading_ref-=1;
    counterL=0;
    counterR=0;
  }
  else if (input == 'X'){
    current_state = STOP;
    stopp(heading_ref);
    counterL=0;
    counterR=0;
  }

  else{
    current_state=STOP;
    stopp(heading_ref);
    counterL=0;
    counterR=0;
  }

  /*switch(current_state){
    FORWARD:
      forward(heading_ref);
      Serial.println("forward");
      
      break;
    REVERSE:
      reverse(heading_ref);
      break;
    STOP:
      stopp(heading_ref);
      break;
    TURN:
      turn(heading_ref);
      break;
    default:
      stopp(heading_ref);
      break;
  }*/


  speedL = constrain(speedL, -255, 255);
  speedR = constrain(speedR, -255, 255);
  //Serial.println(speedL);
  //Serial.println(speedR);
  motorL.drive(speedL);
  motorR.drive(speedR);
  

}

void UpdatePosition(){
  float distance_moved = counterAVG*Distance_constant;
  x = distance_moved *cos(90 - ((180/pi)*heading));
  y = distance_moved * sin(90-((180/pi)*heading));
  counterL=0;
  counterR=0;
  
}


void forward(float heading_refference){
   heading_err = heading - heading_refference;
  //Serial.println("forward");
  //possibly the other way around
  if (heading_err < 5)
  {
    speedL = 255 - abs(Kd*heading_err);
    speedR = 255;
  }
  else if (heading_err > 5)
  {
    speedR = 255 - abs(Kd*heading_err);
    speedL = 255;
  }
  else
  {
    speedR = 255;
    speedL = 255;
  }
}
void reverse(float heading_refference){
   heading_err = heading - heading_refference;
  
  //possibly the other way around
  if (heading_err < 5)
  {
    speedL = -255 + abs(Kd*heading_err);
    speedR = -255;
  }
  else if (heading_err > 5)
  {
    speedR = -255 + abs(Kd*heading_err);
    speedL = -255;
  }
  else
  {
    speedR = 255;
    speedL = 255;
  }
}

void stopp(float heading_refference){
  heading_err = heading - heading_refference;
  if (heading_err < 5)
  {
    speedL = -255/4 ;
    speedR = 255/4;
  }
  else if (heading_err > 5)
  {
    speedR = -255/4 ;
    speedL = 255/4;
  }
  else
  {
    speedR = 0;
    speedL = 0;
  }
}

void turn(float heading_refference){
  heading_err = heading - heading_refference;
  if (heading_err < 2)
  {
    speedL = -255 ;
    speedR = 255;
  }
  else if (heading_err > 2)
  {
    speedR = -255 ;
    speedL = 255;
  }
  else
  {
    speedR = 0;
    speedL = 0;
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
