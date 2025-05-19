/****************************************************************
 *  ReceiverAP + On-board “Vest” Sensor
 *  - SoftAP  : ESPP_Receiver  (channel 1, WPA2 pass “12345678”)
 *  - ESP-NOW : broadcast listener → per-device table
 *  - HTTP    : /devices → JSON
 *  - Vest    : reads MPU-6050 @ SDA13/SCL14 + flex on GPIO16
 ****************************************************************/
#include <WiFi.h>
#include <esp_now.h>
#include <WebServer.h>
#include <Wire.h>
#include <math.h>

// ───────── SoftAP / HTTP config ─────────
static const char*  AP_SSID = "ESPP_Receiver";
static const char*  AP_PSK  = "12345678";
static constexpr int HTTP_PORT   = 80;
static constexpr int MAX_DEVICES = 20;

// ───────── ESB-P frame (88 B) ─────────
typedef struct {
  char deviceId[16];
  char broadcastType[16];
  uint8_t payload[56];
} espMessage;

// ───────── Device table entry ─────────
struct Device {
  String  id;

  // FORM
  bool    hasForm=false;
  String  formJson;

  // HR
  bool    hasHr=false;    int lastHr=0; unsigned long hrTs=0;

  // VMEDIDO
  bool    hasV=false;     float lastV=0;

  // ACCEL
  bool    hasAccel=false; int16_t ax=0,ay=0,az=0;

  // FLEX
  bool    hasFlex=false;  uint16_t flexRes=0;
};
static Device devices[MAX_DEVICES];
static int    deviceCount=0;

// ───────── HTTP server ─────────
WebServer server(HTTP_PORT);

// ───────── Helpers ─────────
int findOrAddDevice(const char* id) {
  for(int i=0;i<deviceCount;i++)
    if(devices[i].id==id) return i;
  if(deviceCount>=MAX_DEVICES) return -1;
  devices[deviceCount].id=id;
  return deviceCount++;
}

// ───────── ESP-NOW callback ─────────
void onDataRecv(const uint8_t*, const uint8_t* data, int len) {
  if(len!=sizeof(espMessage)) return;
  auto *m=(espMessage*)data;
  int idx=findOrAddDevice(m->deviceId);
  if(idx<0) return;
  Device &d=devices[idx];

  /* broadcastType switching */
  if(!strcmp(m->broadcastType,"FORM")){
    d.hasForm=true;
    d.formJson=String("{")+                       // offsets hard-wired
      "\"edad\":\""+String((char*)m->payload+0 ) +"\","+
      "\"altura\":\""+String((char*)m->payload+4 )+"\","+
      "\"peso\":\""+String((char*)m->payload+8 ) +"\","+
      "\"vasos\":\""+String((char*)m->payload+12)+"\","+
      "\"hrs\":\""+String((char*)m->payload+16 ) +"\","+
      "\"nivel\":\""+String((char*)m->payload+20)+"\"}";
  }
  else if(!strcmp(m->broadcastType,"HR")){
    d.hasHr=true;
    d.lastHr=*reinterpret_cast<const int32_t*>(&m->payload[0]);
    d.hrTs=millis();
  }
  else if(!strcmp(m->broadcastType,"VMEDIDO")){
    d.hasV=true;
    d.lastV=*reinterpret_cast<const float*>(&m->payload[0]);
  }
  else if(!strcmp(m->broadcastType,"ACCEL")){
    d.hasAccel=true;
    d.ax=*reinterpret_cast<const int16_t*>(&m->payload[0]);
    d.ay=*reinterpret_cast<const int16_t*>(&m->payload[2]);
    d.az=*reinterpret_cast<const int16_t*>(&m->payload[4]);
  }
  else if(!strcmp(m->broadcastType,"FLEX")){
    d.hasFlex=true;
    d.flexRes=*reinterpret_cast<const uint16_t*>(&m->payload[0]);
  }
}

// ───────── /devices handler ─────────
void handleDevices(){
  String j="{\"devices\":[";
  for(int i=0;i<deviceCount;i++){
    Device &d=devices[i];
    j+="{\"id\":\""+d.id+"\"";
    if(d.hasForm)   j+=",\"FORM\":"+d.formJson;
    if(d.hasHr)     j+=",\"HR\":"+String(d.lastHr);
    if(d.hasV)      j+=",\"VMEDIDO\":"+String(d.lastV,3);
    if(d.hasAccel)  j+=",\"ACCEL\":{\"ax\":"+String(d.ax)+",\"ay\":"+String(d.ay)+",\"az\":"+String(d.az)+"}";
    if(d.hasFlex)   j+=",\"FLEX\":"+String(d.flexRes);
    j+="}";
    if(i<deviceCount-1) j+=",";
  }
  j+="]}";
  server.send(200,"application/json",j);
}

// ───────── Vest sensor (MPU + flex) ─────────
#define SDA_PIN 13
#define SCL_PIN 14
#define MPU_ADDR 0x68
#define FLEX_PIN 16
const float VREF=3.3; const int ADCMAX=4095; const float RPULL=10000;

void initVest(){
  Wire.begin(SDA_PIN,SCL_PIN);
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); Wire.write(0); Wire.endTransmission(); // wake MPU
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
}

void sampleVest(){
  // read accel
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR,6,true);
  int16_t ax=(Wire.read()<<8)|Wire.read();
  int16_t ay=(Wire.read()<<8)|Wire.read();
  int16_t az=(Wire.read()<<8)|Wire.read();

  // read flex
  int rawFlex = analogRead(FLEX_PIN);
  float vFlex = rawFlex*VREF/ADCMAX;
  float rFlex = (vFlex*RPULL)/(VREF-vFlex);

  // update table
  int idx=findOrAddDevice("Vest");
  if(idx<0) return;
  Device &d=devices[idx];
  d.hasAccel=true; d.ax=ax; d.ay=ay; d.az=az;
  d.hasFlex =true; d.flexRes=(uint16_t)rFlex;
}

// ───────── Setup ─────────
void setup(){
  Serial.begin(115200);

  /* SoftAP + ESP-NOW */
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(AP_SSID,AP_PSK,1);
  Serial.printf("AP %s up  %s\n",AP_SSID,WiFi.softAPIP().toString().c_str());

  esp_now_init();
  esp_now_register_recv_cb(onDataRecv);
  esp_now_peer_info_t peer={};
  memset(peer.peer_addr,0xFF,6); peer.channel=1; peer.encrypt=false;
  esp_now_add_peer(&peer);

  /* HTTP */
  server.on("/devices",handleDevices);
  server.begin();

  /* Vest HW */
  initVest();
}

// ───────── Loop ─────────
unsigned long lastVestMs=0;
void loop(){
  server.handleClient();

  unsigned long now=millis();
  if(now-lastVestMs>=200){    // sample every 200 ms
    sampleVest();
    lastVestMs=now;
  }
}
