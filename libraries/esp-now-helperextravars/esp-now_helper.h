#include <esp_now.h>
#include <WiFi.h>

#define SEND_DELAY 150 //150
#define CONTROLLER_ID 1


//structure to hold data to send - can create more structures if different data is to be sent
typedef struct all_struct {
  char x; //movement 
  float allhead[4];
  float allxpos[4];
  float allypos[4]; //position (x,y)
  float testvals[4];
  float debugs[8]
} all_struct;

typedef struct ind_struct {
  int id;
  float err;
  float heading;
  float xpos; 
  float ypos; //x, y 
  int test_val;
  float test1;
  float test2;
} ind_struct;


void send2slave(const uint8_t *mac, esp_now_send_status_t status);
void recfromslave(const uint8_t * mac_addr, const uint8_t *incomingData, int len);

void recfrommaster(const uint8_t * mac, const uint8_t *incoming, int len);
void send2master(const uint8_t *mac, esp_now_send_status_t status);

void setup_esp_now_master();
void master_send(float err, float heading, float xpos, float ypos);

void setup_esp_now_slave();
void slave_send(int id, float err, float heading, float xpos, float ypos, int test_val, int test1, int test2);

void setup_esp_now_controller();
void controller_send();

//controller uses same callback function when sent to master

