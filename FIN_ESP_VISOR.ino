//// ESP32 #3: conectada por USB a la 2da PC (visor 3D)
//// Solo RECIBE por ESP-NOW los angulos que manda la ESP del dron y los
//// imprime por serial como "roll,pitch,yaw". No necesita ninguna MAC de destino.
//// Para ESP32 Arduino core 3.x

#include <esp_now.h>
#include <WiFi.h>

//Misma estructura que en FIN_ESP_DRON.ino y FIN_ESP_PC.ino (el tamano debe coincidir)
struct esp_now_msg {
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
  int16_t yawd;
};

//callback de recepcion (firma del core 3.x)
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *datas, int data_len) {
  if (data_len < (int)sizeof(esp_now_msg)) return;  // ignora paquetes incompletos

  esp_now_msg msg_rec;
  memcpy(&msg_rec, datas, sizeof(msg_rec));

  Serial.print(msg_rec.roll / 10.0, 1);    // roll en grados
  Serial.print(",");
  Serial.print(msg_rec.pitch / 10.0, 1);   // pitch en grados
  Serial.print(",");
  Serial.println(msg_rec.yaw);             // yaw (heading) en grados
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  delay(100);
  Serial.print("MAC de esta placa (3ra ESP): ");
  Serial.println(WiFi.macAddress());   // copiala en esp32_visor_mac[] de FIN_ESP_DRON.ino

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESPNow Init Failed");
    ESP.restart();
  }
  esp_now_register_recv_cb(OnDataRecv);
  Serial.println("ESPNow listo, esperando datos del dron...");
}

void loop() {
  // nada: todo ocurre en el callback
}
