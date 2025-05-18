#include <WiFi.h>
#include <esp_now.h>
#include <WebServer.h>

// — AP + HTTP
static const char* AP_SSID  = "ESP_Receiver";
static const char* AP_PSK   = "12345678";
static const int   HTTP_PORT = 80;
WebServer server(HTTP_PORT);

// Estructura ESB-P
typedef struct __attribute__((packed)) {
  char    deviceId[16];
  char    broadcastType[16];
  uint8_t payload[56];
} espMessage;

// Estado de cada dispositivo
struct Device {
  String id;
  bool   hasVM = false;
  float  ts, Vo, MA, R, Rmin, Rmax;
};
static Device devices[20];
static int    deviceCount = 0;

// Callback ESPNOW
void onDataRecv(const uint8_t *mac, const uint8_t *data, int len) {
  if (len != sizeof(espMessage)) return;
  auto *m = (espMessage*)data;

  // 1) Buscar o crear índice
  int idx=-1;
  for(int i=0;i<deviceCount;i++){
    if(devices[i].id == m->deviceId) { idx=i; break; }
  }
  if (idx<0 && deviceCount<20) {
    idx = deviceCount++;
    devices[idx].id = String(m->deviceId);
  }
  if (idx<0) return;
  Device &d = devices[idx];

  // 2) Si es VMEDIDO
  if (String(m->broadcastType) == "VMEDIDO") {
    float arr[6];
    memcpy(arr, &m->payload[0], sizeof(arr));
    d.ts   = arr[0];
    d.Vo   = arr[1];
    d.MA   = arr[2];
    d.R    = arr[3];
    d.Rmin = arr[4];
    d.Rmax = arr[5];
    d.hasVM = true;
  }
}

// Handler HTTP /devices
void handleDevices(){
  String j = "{\"devices\":[";
  for(int i=0;i<deviceCount;i++){
    auto &d = devices[i];
    j += "{\"id\":\""+d.id+"\"";
    if (d.hasVM) {
      j += ",\"timestamp\":"+String(d.ts,0);
      j += ",\"Vo\":"+String(d.Vo,4);
      j += ",\"MA\":"+String(d.MA,4);
      j += ",\"R\":"+String(d.R,0);
      j += ",\"Rmin\":"+String(d.Rmin,4);
      j += ",\"Rmax\":"+String(d.Rmax,4);
    }
    j += "}";
    if (i<deviceCount-1) j += ",";
  }
  j += "]}";
  server.send(200,"application/json",j);
}

void setup(){
  Serial.begin(115200);
  delay(100);

  // 1) Configuro modo mixto AP+STA UNA SOLA VEZ
  WiFi.mode(WIFI_AP_STA);

  // 2) Arranco el AP en canal 1 (coincide con ESPNOW)
  WiFi.softAP(AP_SSID, AP_PSK, /*channel=*/1);
  IPAddress ip = WiFi.softAPIP();
  Serial.printf("AP \"%s\" started @ %s on channel %d\n",
                AP_SSID, ip.toString().c_str(), 1);

  // 3) Iniciar servidor HTTP
  server.on("/devices", handleDevices);
  server.begin();
  Serial.printf("HTTP server @ port %d\n", HTTP_PORT);

  // 4) Inicializar ESPNOW (sin volver a tocar WiFi.mode)
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error inicializando ESP-NOW");
    return;
  }
  esp_now_register_recv_cb(onDataRecv);
  esp_now_peer_info_t peer = {};
  memset(peer.peer_addr, 0xFF, 6);
  peer.channel = 1;
  peer.encrypt = false;
  esp_now_add_peer(&peer);

  Serial.println("ESPNOW listo para recibir.");
}


void loop(){
  server.handleClient();
}
