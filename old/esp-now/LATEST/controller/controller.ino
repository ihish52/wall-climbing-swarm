#include <esp_now.h>
#include <WiFi.h>

uint8_t masterAddress[] = {0x84,0xCC,0xA8,0x01,0x03,0x20}; //my master firebeetle

//board on robot = {0x30,0x83,0x98,0x53,0xBB,0x24}

typedef struct send_struct {
  int id; //set as 4 for controller
  float err; //used to send int movement
  float heading;
  float pos[2]; //x, y 
} send_struct;

typedef struct rec_slave {
  char x; //movement 
  float allerr[5]; //error
  float allhead[5]; 
  float allpos[5][2]; //position (x,y)
} rec_slave;

send_struct master_data;
rec_slave datarec;

char movement;

//callback function for when data is sent to master
void sendmaster(const uint8_t *mac, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

//also for testing
void afterrec(const uint8_t * mac_addr, const uint8_t *incomingData, int len) {
  memcpy(&datarec, incomingData, sizeof(datarec));
  movement = datarec.x;
//
//  Serial.print("received = ");
//  Serial.println(datarec.allerr[0]);
}

void setup() {
  // put your setup code here, to run once:
  pinMode(36, INPUT); //up
  pinMode(39, INPUT); //reverse
  pinMode(34, INPUT); //right
  pinMode(35, INPUT); //left //35

  Serial.begin(115200);
  WiFi.mode(WIFI_STA); //setting device as wifi station
//
  //initialise ESP-NOW
  if(esp_now_init() != ESP_OK) {
    Serial.println("Error initialising ESP-NOW");
    return;
  }
  
  //register callback function for when message is sent
  esp_now_register_send_cb(sendmaster);
  esp_now_register_recv_cb(afterrec);

  //register master
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, masterAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  
  // Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

  Serial.println("yes");
}


char x = ' ';
int up = 0;
int rev = 0;
int left = 0;
int right = 0;

long timer = millis();  // the last time the output pin was toggled

void loop() {
  
//  Serial.println("ready! :)");
  // put your main code here, to run repeatedly:
  up = digitalRead(36);
  rev = digitalRead(39);
  left = digitalRead(35); //35
  right = digitalRead(34);

  master_data.id = 4; //for controller
    
  if (up == HIGH){
    master_data.err = 'U'; //master_data.err
  }
  else if (rev == HIGH){
    master_data.err = 'D';
  }
  else if (left == HIGH){
    master_data.err = 'L';
  }
  else if (right == HIGH) {
    master_data.err = 'R';
  }
  else {
    master_data.err = 'X';
  }

  if(millis() - timer > 120) {
    esp_err_t result = esp_now_send(masterAddress, (uint8_t *) &master_data, sizeof(master_data));
    timer = millis();
//    Serial.println(master_data.err);
  }
}
