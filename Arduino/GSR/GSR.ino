#include <WiFi.h>
#include <esp_now.h>

// Estructura ESB-P universal
typedef struct __attribute__((packed)) {
  char    deviceId[16];      // MAC
  char    broadcastType[16]; // "VMEDIDO"
  uint8_t payload[56];       // aquí metemos 6 floats (24 bytes) y cero el resto
} espMessage;

// Broadcast a todas las ESPPADRE
static uint8_t broadcastAddress[] = {
  0xFF,0xFF,0xFF,0xFF,0xFF,0xFF
};

// Pines y PWM/ADC como antes
const int pwmPin        = 2, adcPin = 1;
const int pwmFreq      = 160, pwmChannel = 0, pwmRes = 8, duty = 204;

// Promedio móvil
const uint8_t MA_SIZE = 10;
float voBuf[MA_SIZE]; uint8_t voIdx=0, voCnt=0; float voSum=0;

// Constantes de regresión calibrada
const float aReg = 5.046e6, bReg = -1.1404e7;
const float RvalMin_const = 0.1f, RvalMax_const = 0.4f;
unsigned long lastSend = 0;

void setup() {
  // PWM
  ledcSetup(pwmChannel, pwmFreq, pwmRes);
  ledcAttachPin(pwmPin, pwmChannel);
  ledcWrite(pwmChannel, duty);
  // ADC
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  // Wi-Fi STA + ESP-NOW
  WiFi.mode(WIFI_STA);
  esp_now_init();
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, broadcastAddress, 6);
  peer.channel = 1; peer.encrypt = false;
  esp_now_add_peer(&peer);
}

void loop() {
  unsigned long now = millis();
  if (now - lastSend < 400) return;
  lastSend = now;

  // 1) Leer Vo y calcular promedio móvil
  float Vo = analogRead(adcPin) * (3.3f/4095.0f);
  if (voCnt < MA_SIZE) {
    voSum += Vo;
    voBuf[voCnt++] = Vo;
  } else {
    voSum -= voBuf[voIdx];
    voSum += Vo;
    voBuf[voIdx] = Vo;
    voIdx = (voIdx+1) % MA_SIZE;
  }
  float MA = voSum / voCnt;

  // 2) Calcular resistencia R
  float Rme = aReg * Vo + bReg;

  // 3) Empaquetar mensaje
  espMessage msg;
  memset(&msg, 0, sizeof(msg));
  // deviceId = MAC
  String mac = WiFi.macAddress();
  mac.toCharArray(msg.deviceId, 16);
  // broadcastType
  String bt = "VMEDIDO";
  bt.toCharArray(msg.broadcastType, 16);
  // payload: [0..5] = {timestamp, Vo, MA, Rme, RvalMin, RvalMax}
  float arr[6] = { float(now), Vo, MA, Rme, RvalMin_const, RvalMax_const };
  memcpy(&msg.payload[0], arr, sizeof(arr));  // 6*4=24 bytes

  // 4) Enviar por ESP-NOW
  esp_now_send(broadcastAddress, (uint8_t*)&msg, sizeof(msg));
}
