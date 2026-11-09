//// Mario Aguilera Ruiz
//// CICESE 2024

////Code for ESP32 board connected to the PC

#include <esp_now.h>
#include <WiFi.h>

int16_t roll, pitch, yaw, arm, thr, rolld, pitchd, p_roll, i_roll, d_roll;
int roll2, pitch2, yaw2;

//broadcast MAC address
//Replace with the MAC address of your ESP32 board connected to the quadcopter
uint8_t esp32_dron_mac[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

unsigned long lastTime = 0, sampleTime = 100;

//structure for ESP-NOW message
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

//function to initialize ESP-NOW protocol
void InitESPNow() {
  if (esp_now_init() == ESP_OK) {
    Serial.println("ESPNow Init Success");
  }
  else {
    Serial.println("ESPNow Init Failed");
    ESP.restart();
  }
}

//function to send ESP-NOW protocol
void enviar(esp_now_msg msg){
  esp_err_t result = esp_now_send(esp32_dron_mac, (uint8_t*) &msg, sizeof(msg));
}

//callback function to receive ESP-NOW messages
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *datas, int data_len) {  
  esp_now_msg msg_rec;
  //memcpy(&msg_rec, datas, sizeof(datas));
memcpy(&msg_rec, datas, sizeof(msg_rec));

  roll = msg_rec.roll;
  pitch = msg_rec.pitch;
  yaw = msg_rec.yaw;

  roll2 = roll / 10.0;
  pitch2 = pitch / 10.0;
  Serial.print(roll2);
  Serial.print(",");
  Serial.println(pitch2);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  //Setup ESP-NOW
  WiFi.mode(WIFI_STA);
  InitESPNow();
  esp_now_peer_info_t peer;
  peer.channel = 1;
  peer.encrypt = 0;
  memcpy(peer.peer_addr, esp32_dron_mac, 6);
  esp_now_add_peer(&peer);
  esp_now_register_recv_cb(OnDataRecv);

}

void loop() {
//Read data from PC
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');  
    input.trim();
    int index1 = input.indexOf(',');
    int index2 = input.indexOf(',', index1 + 1);
    int index3 = input.indexOf(',', index2 + 1);
    int index4 = input.indexOf(',', index3 + 1);
    int index5 = input.indexOf(',', index4 + 1);
    int index6 = input.indexOf(',', index5 + 1);
    if (index1 != -1 && index2 != -1 && index3 != -1 && index4 != -1 && index5 != -1 && index6 != -1) {
      arm = input.substring(0, index1).toInt();
      thr = input.substring(index1 + 1, index2).toInt();
      rolld = input.substring(index2 + 1, index3).toInt();
      pitchd = input.substring(index3 + 1).toInt();
      p_roll = input.substring(index4 + 1).toInt();
      i_roll = input.substring(index5 + 1).toInt();
      d_roll = input.substring(index6 + 1).toInt();


      
    }
  }


  //send data to the quadcopter every 100ms
  if(millis() - lastTime > sampleTime){
    lastTime = millis();
    esp_now_msg msg;
    msg.arm = arm;
    msg.thr = thr;
    msg.rolld = rolld;
    msg.pitchd = pitchd;
    msg.p_roll = p_roll;
    msg.i_roll = i_roll;
    msg.d_roll = d_roll;

    enviar(msg);

  }

}
