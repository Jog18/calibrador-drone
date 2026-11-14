
//// Mario Aguilera Ruiz
//// CICESE 2024

//// Code for ESP32 board connected to quadcopter
//// Corregido para ESP32 Arduino core 3.x

#include <esp_now.h>
#include <WiFi.h>

//variables declaration

uint8_t PIDitems[30] = {22,20,30,22,20,30,80,40,0,50,0,0,15,0,0,34,14,53,25,33,83,20,0,0,40,0,0,120,45,1}; //PID default values
uint8_t PIDitems2[30];  //variable to store desired PID values
long startTime, elapsedTime, startTime2, elapsedTime2;
bool flag=0;
int16_t roll, pitch, yaw, arm, thr, rolld, pitchd, p_roll, i_roll, d_roll, yawd;
int ch1, ch2, ch3, ch4, ch5; //variables for PWM outputs

//PWM output pins
#define PIN_ROLL  27
#define PIN_PITCH 26
#define PIN_THR   25
#define PIN_YAW   33
#define PIN_ARM   32

//commands for MSP protocol
#define MSP_attitude 108
#define MSP_pid 112
#define MSP_set_pid 202

// ============================================================
//  >>> AQUI VA LA MAC DE LA ESP32 CONECTADA A LA PC <<<
//  (NO es la de esta placa, es la de la otra ESP32)
//  Ejemplo: si la MAC es AA:BB:CC:DD:EE:FF
//  queda:   {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF}
// ============================================================
uint8_t esp32_pc_mac[] = {0x04, 0xB2, 0x47, 0x97, 0x77, 0xA0}; 

// ============================================================
//  >>> MAC DE LA 3RA ESP32 (la conectada a la 2da PC, visor 3D) <<<
//  Mientras este en {0,0,0,0,0,0} el envio al visor queda desactivado.
// ============================================================
uint8_t esp32_visor_mac[] = {0xA0, 0xB7, 0x65, 0x14, 0x3E, 0xF4};
bool visor_activo = false;

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
  int16_t yawd;      // comando de yaw (-50..50), 0 = sin giro
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
  // NULL = envia a todos los peers registrados (ESP-PC y, si esta activo, el visor)
  esp_err_t result = esp_now_send(NULL, (uint8_t*) &msg, sizeof(msg));
}

//callback function to receive ESP-NOW messages
//(firma nueva del core 3.x: recibe esp_now_recv_info_t en lugar de la MAC)
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *datas, int data_len) {
  if (data_len < (int)sizeof(esp_now_msg)) return;  // ignora paquetes incompletos

  esp_now_msg msg_rec;
  memcpy(&msg_rec, datas, sizeof(msg_rec));
  arm = msg_rec.arm;
  thr = msg_rec.thr;
  rolld = msg_rec.rolld;
  pitchd = msg_rec.pitchd;
  yawd = msg_rec.yawd;
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
  Serial.print(PIDitems[2]);
  Serial.print(",");
  Serial.println(yawd);
}

//function to send MSP commands to the Flight Controller
void sendMSP(uint8_t cmd, uint8_t *data1, uint8_t n_bytes) {
    uint8_t checksum = 0;
    Serial2.write((const uint8_t *) "$M<", 3);
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
          break;
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
    checksum = checksum^PIDitems[i];   // el checksum debe calcularse con los mismos datos que se envian
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
  memset(&peer, 0, sizeof(peer));   // inicializa toda la estructura en cero
  peer.channel = 0;                 // 0 = usa el canal actual del WiFi
  peer.encrypt = false;
  memcpy(peer.peer_addr, esp32_pc_mac, 6);
  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("Error al agregar el peer (revisa la MAC)");
  }
  //3ra ESP (visor 3D en la 2da PC): solo si se configuro su MAC
  for (int i = 0; i < 6; i++) {
    if (esp32_visor_mac[i] != 0) visor_activo = true;
  }
  if (visor_activo) {
    memcpy(peer.peer_addr, esp32_visor_mac, 6);
    if (esp_now_add_peer(&peer) != ESP_OK) {
      Serial.println("Error al agregar el peer del visor (revisa la MAC)");
    }
  }

  esp_now_register_recv_cb(OnDataRecv);

  //PWM outputs configuration (core 3.x: ledcAttach(pin, frecuencia, resolucion))
  ledcAttach(PIN_ROLL,  50, 12);  //OutROLL
  ledcAttach(PIN_PITCH, 50, 12);  //OutPITCH
  ledcAttach(PIN_THR,   50, 12);  //OutTHR
  ledcAttach(PIN_YAW,   50, 12);  //OutYAW
  ledcAttach(PIN_ARM,   50, 12);  //On/off

  //start timers
  startTime=millis();
  startTime2=millis();
}

void loop() {
  //calculate desired PWM outputs
  ch1 = round(((rolld+50) * 205) / 100 + 205);
  ch2 = round(((pitchd+50) * 205) / 100 + 205);
  ch3 = round((thr-0)*(410 - 205)/(100 - 0) + 205);
  int yaw_cmd = constrain((int)yawd, -50, 50);   // limita el comando de yaw
  ch4 = ((yaw_cmd + 50) * 205) / 100 + 205;      // yawd = 0 -> 307 (neutro)
  if(arm == 1){
    ch5 = 410;
  }
  else{
    ch5 = 205;
  }

  //write desired PWM outputs (core 3.x: ledcWrite recibe el pin, no el canal)
  ledcWrite(PIN_ROLL,  ch1);
  ledcWrite(PIN_PITCH, ch2);
  ledcWrite(PIN_THR,   ch3);
  ledcWrite(PIN_YAW,   ch4);
  ledcWrite(PIN_ARM,   ch5);

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
    memset(&msg, 0, sizeof(msg));
    msg.roll = roll;
    msg.pitch = pitch;
    msg.yaw = yaw;
    enviar(msg);
    startTime=millis();
  }

  //set desired PID values every 500ms
  elapsedTime2 = millis() - startTime2;
  if(elapsedTime2 >= 500){
    setPID();
    delay(10);
    startTime2 = millis();   // antes estaba en 0, lo que hacia que se enviara en cada ciclo
  }
}
