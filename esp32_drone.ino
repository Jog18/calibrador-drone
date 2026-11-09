
//// Mario Aguilera Ruiz
//// CICESE 2024

//// Code for ESP32 board connected to quadcopter

////Works with esp32 core version 1.0.6
#include <esp_now.h>
#include <WiFi.h>

//variables declaration

uint8_t PIDitems[30] = {22,20,30,22,20,30,80,40,0,50,0,0,15,0,0,34,14,53,25,33,83,20,0,0,40,0,0,120,45,1}; //PID default values
uint8_t PIDitems2[30];  //variavle to store desired PID values
long startTime, elapsedTime, startTime2, elapsedTime2;
bool flag=0;
int16_t roll, pitch, yaw, arm, thr, rolld, pitchd, p_roll, i_roll,d_roll;
int ch1, ch2, ch3, ch4, ch5; //variables for PWM outputs 
//commands for MSP protocol
#define MSP_attitude 108
#define MSP_pid 112
#define MSP_set_pid 202

//broadcast MAC address
//Replace with the MAC address of your ESP32 board connected to the PC.
uint8_t esp32_pc_mac[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

//Structure for ESP-NOW messages
struct esp_now_msg{
  int16_t roll;
  int16_t pitch;
  int16_t yaw;
  int16_t arm;
  int16_t thr;
  int16_t rolld;
  int16_t pitchd;
  int16_t p_roll;
  int16_t i_roll;
  int16_t d_roll;
};

//function for initializing ESP-NOW protocol
void InitESPNow() {
  if (esp_now_init() == ESP_OK) {
    Serial.println("ESPNow Init Success");
  }
  else {
    Serial.println("ESPNow Init Failed");
    ESP.restart();
  }
}

//function to send messages using ESP-NOW protocol
void enviar(esp_now_msg msg){
  esp_err_t result = esp_now_send(esp32_pc_mac, (uint8_t*) &msg, sizeof(msg));
}

//callback function to receive  ESP-NOW messages
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *datas, int data_len) { 
  esp_now_msg msg_rec;
  memcpy(&msg_rec, datas, sizeof(msg_rec));
  arm = msg_rec.arm;
  thr = msg_rec.thr;
  rolld = msg_rec.rolld;
  pitchd = msg_rec.pitchd;
  PIDitems[0] = msg_rec.p_roll;
  PIDitems[1] = msg_rec.i_roll;
  PIDitems[2] = msg_rec.d_roll;

  Serial.print(arm);
  Serial.print(",");
  Serial.print(thr);
  Serial.print(",");
  Serial.print(rolld);
  Serial.print(",");
  Serial.print(pitchd);
  Serial.print(",");
  Serial.print(PIDitems[0]);
  Serial.print(",");
  Serial.print(PIDitems[1]);
  Serial.print(",");
  Serial.println(PIDitems[2]);
}

//function to send MSP commands to the Flight Controller
void sendMSP(uint8_t cmd, uint8_t *data1, uint8_t n_bytes) {
    uint8_t checksum = 0;
    Serial2.write((byte *) "$M<", 3);
    Serial2.write(n_bytes);
    checksum ^= n_bytes;
    Serial2.write(cmd);
    checksum ^= cmd;
    Serial2.write(checksum);

}

//function to read the MSP response
void readMSP_attitude() {
    byte count2 = 0;
    while (Serial2.available()) {
        count2++;
        byte c2 = Serial2.read();
        switch (count2) {
            case 6:
                roll = c2;
                break;
            case 7:
                roll <<= 8;
                roll += c2;
                roll = (roll & 0xFF00) >> 8 | (roll & 0x00FF) << 8; // Reverse the order of bytes
                break;
            case 8:
                pitch = c2;
                break;
            case 9:
                pitch <<= 8;
                pitch += c2;
                pitch = (pitch & 0xFF00) >> 8 | (pitch & 0x00FF) << 8; // Reverse the order of bytes
                break;
            case 10:
                yaw = c2;
                break;
            case 11:
                yaw <<= 8;
                yaw += c2;
                yaw = (yaw & 0xFF00) >> 8 | (yaw & 0x00FF) << 8; // Reverse the order of bytes
                break; 
        }
    }
 
}

//function to read MSP response
void readMSP_pid(){
    byte count2 = 0;
    while (Serial2.available()) {
      count2++;
      byte c2 = Serial2.read();
      switch (count2) {
        case 6:
          PIDitems[0] = c2;
          break;
        case 7:
          PIDitems[1] = c2;
        case 8:
          PIDitems[2] = c2;
          break;
        case 9:
          PIDitems[3] = c2;
          break;
        case 10:
          PIDitems[4] = c2;
          break;
        case 11:
          PIDitems[5] = c2;
          break; 
        case 12:
          PIDitems[6] = c2;
          break;
        case 13:
          PIDitems[7] = c2;
          break;
        case 14:
          PIDitems[8] = c2;
          break;
      }
    }
}

//Function to send desired PID values to Flight Controller using MSP 
void setPID(){
  uint8_t checksum = 0;
  checksum = 30^202;
  for (uint8_t i=0; i<30; i++){
    checksum = checksum^PIDitems2[i];
  }
  Serial2.write('$');
  Serial2.write('M');
  Serial2.write('<');
  Serial2.write(30);
  Serial2.write(MSP_set_pid);
  for (uint8_t i=0; i<30; i++){
    Serial2.write(PIDitems[i]);
  }
  Serial2.write(checksum);
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, 16, 17);  //Configure Serial Communication on GPIO 16 and 17
  //ESP-NOW setup 
  WiFi.mode(WIFI_STA);
  InitESPNow();
  esp_now_peer_info_t peer;
  peer.channel = 1;
  peer.encrypt = 0;
  memcpy(peer.peer_addr, esp32_pc_mac, 6);
  esp_now_add_peer(&peer);
  esp_now_register_recv_cb(OnDataRecv);
  //PWM outputs configuration
  ledcSetup(0,50,12);
  ledcSetup(1,50,12);
  ledcSetup(2,50,12);
  ledcSetup(3,50,12);
  ledcSetup(4,50,12);
  ledcAttachPin(27,0);  //OutROLL
  ledcAttachPin(26,1);  //OutPITCH
  ledcAttachPin(25,2);  //OutTHR
  ledcAttachPin(33,3);  //OutYAW
  ledcAttachPin(32,4);  //On/off
  //start timers
  startTime=millis();
  startTime2=millis();
}

void loop() {
  //calculate desired PWM outputs
  //ch1 = (x1 - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  ch1 = round(((rolld+50) * 205) / 100 + 205); 
  ch2 = round(((pitchd+50) * 205) / 100 + 205); 
  ch3 = round((thr-0)*(410 - 205)/(100 - 0) + 205);//205;
  ch4 = 307;
  if(arm == 1){
    ch5 = 410;
  }
  else{
    ch5 = 205;
  }
  //write desired PWM outputs
  ledcWrite(0,ch1);
  ledcWrite(1,ch2);
  ledcWrite(2,ch3);
  ledcWrite(3,ch4);
  ledcWrite(4,ch5);

  //Read Quadcopter attitude every 60ms
  elapsedTime=millis()-startTime;
  if (elapsedTime>=60){
    uint8_t datad2 = 0;
    uint8_t *data2 = &datad2;
    sendMSP(MSP_attitude, data2, 0);
    delay(10);
    readMSP_attitude();
    delay(10);
    
    //Send ESP-NOW message to the PC
    esp_now_msg msg;
    msg.roll = roll;
    msg.pitch = pitch;
    msg.yaw = yaw;
    enviar(msg);
    startTime=millis();
  }

  //set desired PID values
  elapsedTime2 = millis() - startTime2;
  if(elapsedTime2 >= 500){
    setPID();
    delay(10);
    startTime2 = 0;
  }


}
