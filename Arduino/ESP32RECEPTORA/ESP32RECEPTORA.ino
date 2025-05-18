#include <WiFi.h>
#include <esp_now.h>
#include <WebServer.h>

// ——— AP + HTTP CONFIG ———
static const char* AP_SSID   = "ESP_Receiver";
static const char* AP_PSK    = "12345678";
static const int   HTTP_PORT = 80;
static const int   MAX_DEVICES = 20;

WebServer server(HTTP_PORT);

// ——— MENSAJE GSR ———
typedef struct __attribute__((packed)) {
  char    deviceId[16];
  char    broadcastType[16]; // "VMEDIDO"
  uint8_t payload[56];       // [0..5] = float ts, Vo, MA, R, Rmin, Rmax
} gsrMessage;

// ——— MENSAJE RELOJ ———
typedef struct __attribute__((packed)) {
  char deviceId[16];
  char broadcastType[8];     // "FORM" o "HR"
  char edad[4], altura[4], peso[4];
  char vasos[4], hrs[4], nivel[16];
  int  hr;
} relojMessage;

// ——— ESTADO UNIFICADO POR DISPOSITIVO ———
struct Device {
  String   id;
  // GSR
  bool     hasVM = false;
  float    ts, Vo, MA, R, Rmin, Rmax;
  // RELOJ
  bool     hasForm = false;
  String   formJson;
  bool     hasHr   = false;
  int      lastHr  = 0;
  unsigned long hrTs = 0;
};

static Device devices[MAX_DEVICES];
static int    deviceCount = 0;

// ——— CALLBACK ESPNOW ———
void onDataRecv(const uint8_t *mac, const uint8_t *data, int len) {
  // Buscar o añadir dispositivo
  // (todos los mensajes llevan deviceId al inicio)
  // primero convertimos deviceId a String
  char idbuf[17]; 
  memcpy(idbuf, data, 16); 
  idbuf[16] = '\0';
  String devId = String(idbuf);

  int idx = -1;
  for (int i = 0; i < deviceCount; i++) {
    if (devices[i].id == devId) { idx = i; break; }
  }
  if (idx < 0 && deviceCount < MAX_DEVICES) {
    idx = deviceCount++;
    devices[idx].id = devId;
  }
  if (idx < 0) return;
  Device &d = devices[idx];

  // Decidir según tamaño de mensaje
  if (len == sizeof(gsrMessage)) {
    // Mensaje GSR
    auto *m = (gsrMessage*)data;
    // extraer floats
    float arr[6];
    memcpy(arr, m->payload, sizeof(arr));
    d.ts   = arr[0];
    d.Vo   = arr[1];
    d.MA   = arr[2];
    d.R    = arr[3];
    d.Rmin = arr[4];
    d.Rmax = arr[5];
    d.hasVM = true;
  }
  else if (len == sizeof(relojMessage)) {
    // Mensaje RELOJ
    auto *r = (relojMessage*)data;
    String bt = String(r->broadcastType);
    if (bt == "FORM") {
      d.hasForm = true;
      // construir JSON del formulario
      d.formJson = String("{") +
        "\"edad\":\""   + String(r->edad)   + "\"," +
        "\"altura\":\"" + String(r->altura) + "\"," +
        "\"peso\":\""   + String(r->peso)   + "\"," +
        "\"vasos\":\""  + String(r->vasos)  + "\"," +
        "\"hrs\":\""    + String(r->hrs)    + "\"," +
        "\"nivel\":\""  + String(r->nivel)  + "\"" +
      "}";
    }
    else if (bt == "HR") {
      d.hasHr  = true;
      d.lastHr = r->hr;
      d.hrTs   = millis();
    }
  }
}

// ——— HTTP HANDLER /devices ———
void handleDevices() {
  String j = "{\"devices\":[";
  for (int i = 0; i < deviceCount; i++) {
    Device &d = devices[i];
    j += "{\"id\":\"" + d.id + "\"";
    // RELOJ
    if (d.hasForm) j += ",\"FORM\":" + d.formJson;
    if (d.hasHr)   j += ",\"HR\":"   + String(d.lastHr);
    // GSR
    if (d.hasVM) {
      j += ",\"timestamp\":" + String(d.ts,0);
      j += ",\"Vo\":"        + String(d.Vo,4);
      j += ",\"MA\":"        + String(d.MA,4);
      j += ",\"R\":"         + String(d.R,0);
      j += ",\"Rmin\":"      + String(d.Rmin,4);
      j += ",\"Rmax\":"      + String(d.Rmax,4);
    }
    j += "}";
    if (i < deviceCount - 1) j += ",";
  }
  j += "]}";
  server.send(200, "application/json", j);
}

// ——— SETUP y LOOP ———
void setup() {
  Serial.begin(115200);
  delay(100);

  // 1) Modo mixto AP+STA
  WiFi.mode(WIFI_AP_STA);

  // 2) Iniciar AP en canal 1 (coincide con ESPNOW)
  WiFi.softAP(AP_SSID, AP_PSK, /*channel=*/1);
  IPAddress ip = WiFi.softAPIP();
  Serial.printf("AP \"%s\" @ %s  ch=1\n",
                AP_SSID, ip.toString().c_str());

  // 3) HTTP
  server.on("/devices", handleDevices);
  server.begin();
  Serial.printf("HTTP @ port %d\n", HTTP_PORT);

  // 4) ESPNOW
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

void loop() {
  server.handleClient();
}
