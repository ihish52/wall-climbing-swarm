#include <esp_now.h>
#include <WiFi.h>

uint8_t masterAddress[] = {0x84,0xCC,0xA8,0x01,0x03,0x20};

//Structure to hold received data
//Must match the sender structure
typedef struct rec_struct {
  char x;
} rec_struct;

//structure to send message to master
typedef struct struct_master {
  int id;
  char x;
} struct_master;

rec_struct rec_data;
struct_master master_data;

//callback function for when data is received
void afterrec(const uint8_t * mac, const uint8_t *incoming, int len) {
  
  //copy data from incoming to data struct
  memcpy(&rec_data, incoming, sizeof(rec_data));

  //print data
//  Serial.print("Bytes received: ");
//  Serial.println(len);


//  Serial.print("Data Received: ");
  Serial.println(rec_data.x);

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
  //esp_now_register_send_cb(sendmaster);

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
}

void loop() {
  // put your main code here, to run repeatedly:
  master_data.id = 2;
  master_data.x = 'd';

  //esp_err_t result = esp_now_send(masterAddress, (uint8_t *) &master_data, sizeof(master_data));
 
//  if (result == ESP_OK) {
//    Serial.println("Sent :D");
//  }
//  else {
//    Serial.println("Oh no :(");
//  }







  
}
