#include <esp_now.h>
#include <WiFi.h>

uint8_t slaveAddress1[] = {0x7C,0x9E,0xBD,0x48,0xAD,0x64}; 
uint8_t slaveAddress2[] = {0x7C,0x9E,0xBD,0x49,0x00,0x04}; 
uint8_t slaveAddress3[] = {0x7C,0x9E,0xBD,0x48,0xB5,0x04}; 
uint8_t contrAddress[] = {0x94,0x3C,0xC6,0x08,0x13,0x04};

//structure to hold data to send - can create more structures if different data is to be sent
typedef struct send_struct {
  char x; //movement 
  float allerr[5]; //error
  float allhead[5]; 
  float allpos[5][2]; //position (x,y)
} send_struct;

typedef struct rec_slave {
  int id;
  float err;
  float heading;
  float pos[2]; //x, y 
} rec_slave;

send_struct datasend;
rec_slave datarec;

//create structure to hold data from all boards
rec_slave slave1;
rec_slave slave2;
rec_slave slave3;

rec_slave allslaves[3] = {slave1,slave2,slave3};

//checks if slaves received data
void aftersend(const uint8_t *mac, esp_now_send_status_t status) {
  char macstr[18];
//  Serial.print(" send status:\t");
//  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Yay :D":"Oh no :(");
//  Serial.println("sent");
}

void afterrec(const uint8_t * mac_addr, const uint8_t *incomingData, int len) {
  //get slave mac address
  char macStr[18];


//  Serial.print("Packet received from: ");
//  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
//           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  

  //copy received data into own struct 
  memcpy(&datarec, incomingData, sizeof(datarec));

  if (datarec.id == 4){
//    Serial.print("received: ");
    datasend.x = datarec.err; //movement command
//    Serial.println(datasend.x);
  }

  //updating data
  else{
    datasend.allerr[datarec.id] = datarec.err;
    datasend.allhead[datarec.id] = datarec.heading;
    datasend.allpos[datarec.id][0] = datarec.pos[0]; 
    datasend.allpos[datarec.id][1] = datarec.pos[1]; 
  }
}

long timer = millis();

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA); //setting device as wifi station

  //initialise ESP-NOW
  if(esp_now_init() != ESP_OK) {
    Serial.println("Error initialising ESP-NOW");
    return;
  }
  
  //register callback function for when message is sent
  esp_now_register_send_cb(aftersend);
  esp_now_register_recv_cb(afterrec);

  //register slaves
  esp_now_peer_info_t slaveinfo;
  slaveinfo.channel = 0;
  slaveinfo.encrypt = false;

  //register first slave
  memcpy(slaveinfo.peer_addr, slaveAddress1, 6);
  if (esp_now_add_peer(&slaveinfo) != ESP_OK) {
    Serial.println("Failed to connect to slave 1");
    return;
  }
  //register second slave
  memcpy(slaveinfo.peer_addr, slaveAddress2, 6);
  if (esp_now_add_peer(&slaveinfo) != ESP_OK) {
    Serial.println("Failed to connect to slave 2");
    return;
  }

  //register third slave
  memcpy(slaveinfo.peer_addr, slaveAddress3, 6);
  if (esp_now_add_peer(&slaveinfo) != ESP_OK) {
    Serial.println("Failed to connect to slave 3");
    return;
  }

  //register controller
  memcpy(slaveinfo.peer_addr, contrAddress, 6);
  if (esp_now_add_peer(&slaveinfo) != ESP_OK) {
    Serial.println("Failed to connect to controller");
    return;
  }

}

//datasend.x = 'N';

int t = 1;

void loop() {
  // put your main code here, to run repeatedly
  t = t += 1;
  
  
  datasend.allhead[0] = 9;
  datasend.allpos[0][0] = 10;
  datasend.allpos[0][1] = 11;
  datasend.allerr[0] = t; //12
  
//  Serial.print("positions-x: ");  
//  Serial.print(datasend.allpos[0][0]);
//  Serial.print(datasend.allpos[1][0]);
//  Serial.println(datasend.allpos[2][0]);
//  
  if(millis() - timer > 120){  //75 
    esp_err_t result = esp_now_send(0, (uint8_t *) &datasend, sizeof(send_struct)); //first parameter 0
    timer = millis();
//    Serial.print("Send: ");
//    Serial.println(datasend.allerr[0]);
  }

  //the receiver can be specified using first parameter - '0' means all
  //specify which data structure used as well
  //can check result for each by specifying which esp_err_t variable you want to checknnn
  
  
}
