// code for the master

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
// #include "BluetoothSerial.h"

#include <esp_now.h>
#include <WiFi.h>

// //BT serial comms
// BluetoothSerial ESP_BT;
char input,previnput;
//BT command strings

#define FULL_SPEED 255
#define Distance_measure 36.25
#define Encoder_ticks 10000
#define Distance_constant Distance_measure/Encoder_ticks
#define MATH_PI 3.141592653589793

#define MAX_ROBOT_NUM 4
#define calibrate_ticks 5000

#define PRINT_TIME 1000
long print_timer = millis();

bool calibrate = false;
float positions[4][4] = {{0,0,0,0},{-1,-1,-1,-1},{-1,-1,-1,-1},{-1,-1,-1,-1}};
uint32_t calibrate_timer = millis();
int test_counter = 0;

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

typedef struct rec_slave {
  int id;
  float err;
  float heading;
  float pos[2];
} rec_slave;

typedef struct send_struct {
  int xpos;
  int ypos;
} send_struct;

rec_slave datarec;
send_struct datasend;

uint8_t contrAddr[] = {0x94,0x3C,0xC6,0x08,0x13,0x04};

//callback function for when data is sent to master
void sendmaster(const uint8_t *mac, esp_now_send_status_t status) {
  //Serial.print("\r\nLast Packet Send Status:\t");
  //Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void afterrec(const uint8_t * mac_addr, const uint8_t *incomingData, int len){
  memcpy(&datarec, incomingData, sizeof(datarec));
  //Serial.println("hello");
  if (datarec.id == 4){
    input = datarec.err;
    //Serial.println(input);
  }
}

void setup() {
  
  Wire.begin();
  Wire.setClock(400000);
  Serial.begin(9600); //9600
  Serial.println("START");
  //while (!Serial);

  //setup gyro acc
  DMP.DMP_setup();
  Serial.println("START2");

  //setup motors and driver
  setupMotors();
  Serial.println("START3");

  // //serial comms
  // ESP_BT.begin("ESP32_plsfindme"); //bluetooth device name

  //setup ESP-NOW
  WiFi.mode(WIFI_STA);

  if(esp_now_init() != ESP_OK){
    Serial.println("Error :(");
    return;
  }

  esp_now_register_recv_cb(afterrec);
  esp_now_register_send_cb(sendmaster);

  //register master
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, contrAddr, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if(esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Error adding peer");
    return;
  }

  Serial.println("START4");

  //interrupt
  attachInterrupt(outputA1, isr_a, CHANGE);
  attachInterrupt(outputA2, isr_a, CHANGE);


}

void isr_a()
{
  updateEnc();
  //Serial.println("updating enc");  
}

bool fw_flag = false;
long timer1 = millis();

void loop() {
  
  //for testing with controller
  datasend.xpos = x;
  datasend.ypos = y;
  
  if (millis() - print_timer > PRINT_TIME)
  {
    Serial.print("input:");
    Serial.println(input);
    Serial.print("x: ");
    Serial.print(datasend.xpos);
    Serial.print(" y: ");
    Serial.print(datasend.ypos);

    print_enc();

    print_timer = millis();
  }
  

  if(millis() - timer1 > 75){  //75 
    esp_err_t result = esp_now_send(0, (uint8_t *) &datasend, sizeof(send_struct));
//    ESP_BT.write(datasend.x);
    timer1 = millis();
    //print moved to print timer above
  }
  
  //update position here commented out for now
  /*if ((current_state == FORWARD)||(current_state == REVERSE)){
    UpdatePosition();
  }*/

  //update encoder counters LR
  //updateEnc();

  //update position
  

  //calculate average of encoder counters LR
  counterAVG = ((counterR)-(counterL))/2.0;
   //read data from BT controller
  // if (ESP_BT.available())
  // {
  //   input = ESP_BT.read();
  // }

  //store current heading in array from DMP
  DMP.ypr_pitch_bound(ypr[0], ypr[1], ypr[2]);
  heading = ypr[2];
  heading_error = heading - heading_ref;
  
  //PD controller
  timer_PID_prev = timer_PID;
  timer_PID = millis();
  elapsed_timer_PID = timer_PID - timer_PID_prev;

  //calibrate code deactivated because calibrate bool initialized to false
  if (calibrate == true)
  {
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

        positions[test_counter+1][1] = 0;
        test_counter++;   
      }
      
    }
  }

  else
  {
  if(input == 'U'){
    current_state=FORWARD;
    if(previnput !='U'){
      //counterL = 0;
      //counterR = 0;
      //counterAVG = 0;
    }
    //fw_flag = true;
    forward();
  }
  else if (input == 'D'){
    current_state=REVERSE;
    if(previnput !='D'){
      //counterL = 0;
      //counterR = 0;
      //counterAVG = 0;
    }
    reverse();
  }
  else if((input == 'L')&&(previnput!='L')){
    current_state = TURN;
    heading_ref -= 5;
    if(heading_ref < -179)
      heading_ref = 180;
  }
  else if((input == 'R')&&(previnput!='R')){
    current_state = TURN;
    heading_ref += 5;
    if(heading_ref > 180)
      heading_ref = -179;  
  }
  else{
    current_state=STOP;
    //fw_flag=false;
      stopp();
  }

  //reset encoder ticks and update position after forward or backward movement stops only
  if (input == 'X' && previnput == 'U') UpdatePosition();
  else if (input == 'X' && previnput == 'D') UpdatePosition();
  else if (input == 'X' && previnput == 'L') ResetEnc(); //reset enc without updating position
  else if (input == 'X' && previnput == 'R') ResetEnc();

  previnput=input;

  /*if(fw_flag == true){
    forward();
    print_enc();
    if(counterAVG * Distance_constant > 10){
      fw_flag = false;
      counterL = 0;
      counterR = 0;
    }
  }*/

  }

  turn();

  driveL = constrain((200*speedL + controlL), -255, 255);
  driveR = constrain((-200*speedR + controlR), -255, 255);
  
  motorL.drive(driveL);
  motorR.drive(driveR);
  
  heading_error_prev = heading_error;
  //UpdatePosition();
  
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
  y += distance_moved *cos((MATH_PI/180)*(heading));
  x += distance_moved * sin((MATH_PI/180)*heading);

  //uncomment again when debugging is done
  counterL=0;
  counterR=0;
  counterAVG=0;

  //prints performed in print timer in loop
  //Serial.print("X = ");
  //Serial.println(x);
  //Serial.print("Y = ");
  //Serial.println(y);
  positions[datarec.id][0] = x;
  positions[datarec.id][1] = y;
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
