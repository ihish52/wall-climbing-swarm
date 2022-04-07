#include <esp_now.h>
#include <WiFi.h>

uint8_t masterAddress[] = {0x84,0xCC,0xA8,0x01,0x03,0x20};

//Structure to hold received data
//Must match the sender structure
typedef struct rec_struct {
  char x;
  float allerr[5];
  float allhead[5];
  float allpos[5][2];
} rec_struct;

//structure to send message to master
typedef struct struct_master {
  int id;
  float err;
  float heading;
  float pos[2];
} struct_master;

rec_struct rec_data;
struct_master master_data;

float allposi[5][2], allerro[5], allheadi[5];
char x;

//callback function for when data is received
void afterrec(const uint8_t * mac, const uint8_t *incoming, int len) {
//  Serial.println("rec");
  //copy data from incoming to data struct
  memcpy(&rec_data, incoming, sizeof(rec_data));
  memcpy(allposi, rec_data.allpos, sizeof(rec_data.allpos));
  memcpy(allerro, rec_data.allerr, sizeof(rec_data.allerr));
  memcpy(allheadi, rec_data.allhead, sizeof(rec_data.allhead));

  x = rec_data.x; //movement

//  Serial.print("received: "); 
//  Serial.println(allerro[0]); //x
//  c += 1;
}

//callback function for when data is sent to master
void sendmaster(const uint8_t *mac, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA); //setting device as wifi station

  //initialise ESP-NOW
  if(esp_now_init() != ESP_OK) {
    Serial.println("Error initialising ESP-NOW");
    return;
  }
  
  //register callback function for when message is sent
  esp_now_register_recv_cb(afterrec);
  esp_now_register_send_cb(sendmaster);

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

  pinMode(2,OUTPUT);
}

long timer = millis();

void loop() {
  // put your main code here, to run repeatedly:
  //COM10 - pink (2), COM7 - blue (1)
  
  master_data.id = 2;
  master_data.heading = 5;
  master_data.pos[0] = 6;
  master_data.pos[1] = 7;
  master_data.err = 8;

  if(millis() - timer > 75) {
    esp_err_t result = esp_now_send(masterAddress, (uint8_t *) &master_data, sizeof(master_data));
    timer = millis();
  }


//  Serial.println("position-x: ");
//  Serial.println(allposi[0][0]);
//  Serial.print(allposi[1][0]);
//  Serial.println(allposi[2][0]);

}
