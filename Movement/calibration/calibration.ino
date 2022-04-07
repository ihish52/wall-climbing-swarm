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

//BT serial comms
BluetoothSerial ESP_BT;
char input;
//BT command strings

#define FULL_SPEED 255
#define Distance_measure 100
#define Encoder_ticks 100
#define Distance_constant Distance_measure/Encoder_ticks
#define MATH_PI 3.141592653589793

#define MAX_ROBOT_NUM 4
#define calibrate_ticks 5000

bool calibrate = true;
float positions = {{0,0,0,0},{-1,-1,-1,-1},{-1,-1,-1,-1},{-1,-1,-1,-1}};
uint32_t calibrate_timer = millis();
void set_robot_num();
int robot_num = 0;

DMP_helper DMP;
float ypr[3];
float heading = 0;
float x = 0 , y = 0;
float heading_ref = 0;
float heading_error, heading_error_prev = 0;

//Motor direction offsets
const int offsetA = 1;
const int offsetB = 1;

//Motor definitions and variables
float Kp = 5;
float Kd = 0.0008;
Motor motorL = Motor(AIN2, AIN1, PWMA, offsetA, STBY, PWM_CH_A);
Motor motorR = Motor(BIN1, BIN2, PWMB, offsetB, STBY, PWM_CH_B);

//Encoders
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
uint32_t timer = millis();

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
  Serial.println("START");
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
  if (calibrate == true){
    set_robot_num(); // needs to know which number robot it is so the right element in the position array can be updated
    if (millis() - calibrate_timer > 3000){
      //check y position of each robot
      if (positions[0][1] == 0 || positions[1][1] == 0 || positions[2][1] == 0 || positions[3][1] == 0) 
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
        if (positions[3][1] != -1) calibrate = false;
      
      }
      
    }
  }
  
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

  //store current heading in array from DMP
  DMP.ypr_pitch_bound(ypr[0], ypr[1], ypr[2]);
  heading = ypr[2];
  heading_error = heading - heading_ref;
  
  //PD controller
  timer_PID_prev = timer_PID;
  timer_PID = millis();
  elapsed_timer_PID = timer_PID - timer_PID_prev;
  
  if(input == 'U'){
    current_state=FORWARD;
    forward();
  }
  else if (input == 'D'){
    current_state=REVERSE;
    reverse();
  }
  else if(input == 'L'){
    current_state = TURN;
    heading_ref -= 2;
  }
  else if(input == 'R'){
    current_state = TURN;
    heading_ref += 2;
  }
  else{
    current_state=STOP;
    stopp();
  }
  
  turn();

  driveL = constrain((200*speedL + controlL), -255, 255);
  driveR = constrain((-200*speedR + controlR), -255, 255);
  
  motorL.drive(driveL);
  motorR.drive(driveR);
  
  heading_error_prev = heading_error;
  
  Serial.println(heading_ref);


}

void UpdatePosition(){
  float distance_moved = counterAVG*Distance_constant;
  x = distance_moved *cos(90 - ((180/MATH_PI)*heading));
  y = distance_moved * sin(90-((180/MATH_PI)*heading));
  counterL=0;
  counterR=0;
  positions[robot_num][0] = x;
  positions[robot_num][1] = y;
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

	controlR = Kp*heading_error + Kd*(heading_error - heading_error_prev);
	controlL = -controlR;

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

void set_robot_num(){
  if ((positions[1][1] && positions[2][1] && positions[3][1]) == -1) robot_num = 0;
  else if (((positions[2][1] && positions[3][1]) == -1) && (positions[0][1] > 0)) robot_num = 1;
  else if ((positions[3][1] == -1) && ((positions[0][1] && positions[0][2]) > 0)) robot_num = 2;
  else if ((positions[0][1] && positions[1][1] && positions[2][1]) > 0) robot_num = 3;
}
