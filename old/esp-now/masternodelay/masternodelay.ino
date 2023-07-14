#include <esp_now.h>
#include <WiFi.h>
#include "BluetoothSerial.h"

uint8_t slaveAddress1[] = {0x7C,0x9E,0xBD,0x48,0xAD,0x64};
uint8_t slaveAddress2[] = {0x7C,0x9E,0xBD,0x49,0x00,0x04}; 
uint8_t slaveAddress3[] = {0x7C,0x9E,0xBD,0x48,0xB5,0x04}; 

//structure to hold data to send - can create more structures if different data is to be sent
typedef struct send_struct {
  char x;
} send_struct;

typedef struct rec_slave {
  int id;
  char x;
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
  Serial.print(" send status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Yay :D":"Oh no :(");
}

void afterrec(const uint8_t * mac_addr, const uint8_t *incomingData, int len) {
  //get slave mac address
  char macStr[18];
  Serial.print("Packet received from: ");
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.println(macStr);

  //copy received data into own struct 
  memcpy(&datarec, incomingData, sizeof(datarec));
  Serial.printf("Board ID %u: %u bytes\n", datarec.id, len);
  
  // Update the structures with the new incoming data
  allslaves[datarec.id-1].x = datarec.x;
  Serial.printf("x value: ");
  Serial.println(allslaves[datarec.id-1].x);
}

//serial comms
BluetoothSerial ESP_BT;
char input;

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
  //esp_now_register_recv_cb(afterrec);

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

  //serial comms
  ESP_BT.begin("ESP32_plsfindme"); //bluetooth device name
  Serial.println("You can pair now :)");
}

//datasend.x = 'N';

void loop() {
  // put your main code here, to run repeatedly:

  if (ESP_BT.available())
  {
    datasend.x = ESP_BT.read();
  }

  if(millis() - timer > 35){
    esp_err_t result = esp_now_send(0, (uint8_t *) &datasend, sizeof(send_struct));
    ESP_BT.write(datasend.x);
    timer = millis();
  }

  //the receiver can be specified using first parameter - '0' means all
  //specify which data structure used as well
  //can check result for each by specifying which esp_err_t variable you want to checknnn
  
  
}
