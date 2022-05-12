#include <esp_now.h>
#include <WiFi.h>

#define SEND_DELAY 150 //150
#define CONTROLLER_ID 1


//structure to hold data to send - can create more structures if different data is to be sent
typedef struct all_struct {
  char x; //movement 
  float allerr[5]; //error
  float allhead[5];
  float allxpos[5];
  float allypos[5]; //position (x,y)
} all_struct;

typedef struct ind_struct {
  int id;
  float err;
  float heading;
  float xpos; 
  float ypos; //x, y 
} ind_struct;


void send2slave(const uint8_t *mac, esp_now_send_status_t status);
void recfromslave(const uint8_t * mac_addr, const uint8_t *incomingData, int len);

void recfrommaster(const uint8_t * mac, const uint8_t *incoming, int len);
void send2master(const uint8_t *mac, esp_now_send_status_t status);

void setup_esp_now_master();
void master_send(float err, float heading, float xpos, float ypos);

void setup_esp_now_slave();
void slave_send(int id, float err, float heading, float xpos, float ypos);

void setup_esp_now_controller();
void controller_send();

//controller uses same callback function when sent to master

