#include "esp-now_helper.h"		

//add controller address
uint8_t slaveAddress1[] = {0x7C,0x9E,0xBD,0x48,0xAD,0x64}; 
uint8_t slaveAddress2[] = {0x94,0x3C,0xC6,0x08,0x13,0x04}; 
uint8_t slaveAddress3[] = {0x7C,0x9E,0xBD,0x48,0xB5,0x04};
uint8_t contrAddress[] =  {0x7C,0x9E,0xBD,0x49,0x00,0x04};  //{0x94,0x3C,0xC6,0x08,0x13,0x04};

uint8_t masterAddress[] = {0x30,0x83,0x98,0x53,0xBB,0x24};//{0x84,0xCC,0xA8,0x01,0x03,0x20};

//fix struct for controller, master and slave
all_struct send_slave;
all_struct rec_master; 
ind_struct rec_slave;
ind_struct send_master; 

/* //create structure to hold data from all boards
rec_slave slave1;
rec_slave slave2;
rec_slave slave3;
rec_slave allslaves[3] = {slave1,slave2,slave3}; */

long timer_master = millis();
long timer_slave = millis();
long timer_cont = millis();

//from controller code
int up = 0;
int rev = 0;
int left = 0;
int right = 0;

//from slave code
extern float allxposi[5], allyposi[5], allerro[5], allheadi[5];
extern char x;

//checks if slaves received data
void send2slave(const uint8_t *mac, esp_now_send_status_t status) {
  char macstr[18];
/*   Serial.print(" send status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Yay :D":"Oh no :("); */
//  Serial.println("sent");
}

void recfromslave(const uint8_t * mac_addr, const uint8_t *incomingData, int len) {

  //copy received data into own struct 
  memcpy(&rec_slave, incomingData, sizeof(rec_slave));

  if (rec_slave.id == 4) {
    send_slave.x = rec_slave.err; //movement command
	/* Serial.println(send_slave.x); */
  }

  //updating data
  else {
    send_slave.allerr[rec_slave.id] = rec_slave.err;
    send_slave.allhead[rec_slave.id] = rec_slave.heading;
    send_slave.allxpos[rec_slave.id] = rec_slave.xpos; 
    send_slave.allypos[rec_slave.id] = rec_slave.ypos; 
  }
}

void setup_esp_now_master()
{
	/* Serial.begin(115200); */
	WiFi.mode(WIFI_STA); //setting device as wifi station

  //initialise ESP-NOW
  if(esp_now_init() != ESP_OK) {
    /* Serial.println("Error initialising ESP-NOW"); */
    return;
  }
  
  //register callback function for when message is sent
  esp_now_register_send_cb(send2slave);
  esp_now_register_recv_cb(recfromslave);

  //register slaves
  esp_now_peer_info_t slaveinfo;
  slaveinfo.channel = 0;
  slaveinfo.encrypt = false;

  //register first slave
  memcpy(slaveinfo.peer_addr, slaveAddress1, 6);
  if (esp_now_add_peer(&slaveinfo) != ESP_OK) {
    /* Serial.println("Failed to connect to slave 1"); */
    return;
  }
  //register second slave
  memcpy(slaveinfo.peer_addr, slaveAddress2, 6);
  if (esp_now_add_peer(&slaveinfo) != ESP_OK) {
    /* Serial.println("Failed to connect to slave 2"); */
    return;
  }

  //register third slave
  memcpy(slaveinfo.peer_addr, slaveAddress3, 6);
  if (esp_now_add_peer(&slaveinfo) != ESP_OK) {
    /* Serial.println("Failed to connect to slave 3"); */
    return;
  }
  
  //register controller
  memcpy(slaveinfo.peer_addr, contrAddress, 6);
  if (esp_now_add_peer(&slaveinfo) != ESP_OK) {
    /* Serial.println("Failed to connect to controller"); */
    return;
  }
}


void master_send(float err, float heading, float xpos, float ypos)
{
  send_slave.allerr[0] = err;
  send_slave.allhead[0] = heading;
  send_slave.allxpos[0] = xpos;
  send_slave.allypos[0] = ypos;
  
  
  if(millis() - timer_master > 120){  //75 
    esp_err_t result = esp_now_send(0, (uint8_t *) &send_slave, sizeof(send_slave));
    timer_master = millis();
  }
}

//functions from slave code

//callback function for when data is received

void recfrommaster(const uint8_t * mac, const uint8_t *incoming, int len) {
//  Serial.println("rec");
  //copy data from incoming to data struct
  memcpy(&rec_master, incoming, sizeof(rec_master));
  memcpy(allxposi, rec_master.allxpos, sizeof(rec_master.allxpos));
  memcpy(allyposi, rec_master.allypos, sizeof(rec_master.allypos));
  memcpy(allerro, rec_master.allerr, sizeof(rec_master.allerr));
  memcpy(allheadi, rec_master.allhead, sizeof(rec_master.allhead));

  x = rec_master.x; //movement
  /* Serial.println(x); */

}

//callback function for when data is sent to master
void send2master(const uint8_t *mac, esp_now_send_status_t status) {
//  Serial.print("\r\nLast Packet Send Status:\t");
//  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void setup_esp_now_slave()
{
	/* Serial.begin(115200); */
	WiFi.mode(WIFI_STA); //setting device as wifi station

  //initialise ESP-NOW
  if(esp_now_init() != ESP_OK) {
    /* Serial.println("Error initialising ESP-NOW"); */
    return;
  }
  
  //register callback function for when message is sent
  esp_now_register_recv_cb(recfrommaster);
  esp_now_register_send_cb(send2master);

  //register master
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, masterAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  
  // Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    /* Serial.println("Failed to add peer"); */
    return;
  }
}

void slave_send(int id, float err, float heading, float xpos, float ypos)
{
  send_master.id = id;
  send_master.xpos = xpos;
  send_master.ypos = ypos;
  send_master.err = err;
  send_master.heading = heading;

  if(millis() - timer_slave > 120) {
    esp_err_t result = esp_now_send(masterAddress, (uint8_t *) &send_master, sizeof(send_master));
    timer_slave = millis();
  }
}

//code from controller

//setup for controller
void setup_esp_now_controller() {
	
  pinMode(26, INPUT); //up - 36
  pinMode(25, INPUT); //reverse - 39
  pinMode(33, INPUT); //right - 34
  pinMode(32, INPUT); //left - 35

  /* Serial.begin(115200); */
  WiFi.mode(WIFI_STA); //setting device as wifi station
//
  //initialise ESP-NOW
  if(esp_now_init() != ESP_OK) {
	/* Serial.println("Error initialising ESP-NOW"); */
	return;
  }
  
  //register callback function for when message is sent
  esp_now_register_send_cb(send2master);
  esp_now_register_recv_cb(recfrommaster);

  //register master
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, masterAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  
  // Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
	/* Serial.println("Failed to add peer"); */
	return;
  }
}

void controller_send() {
	up = digitalRead(26);
  rev = digitalRead(25);
  left = digitalRead(32); //35
  right = digitalRead(33);

  send_master.id = 4; //for controller
    
  if (up == HIGH){
    send_master.err = 'U'; //send_master.err
  }
  else if (rev == HIGH){
    send_master.err = 'D';
  }
  else if (left == HIGH){
    send_master.err = 'L';
  }
  else if (right == HIGH) {
    send_master.err = 'R';
  }
  else {
    send_master.err = 'X';
  }

  if(millis() - timer_cont > 120) {
    esp_err_t result = esp_now_send(masterAddress, (uint8_t *) &send_master, sizeof(send_master));
    timer_cont = millis();
    /* Serial.println(send_master.err); */
  }
}