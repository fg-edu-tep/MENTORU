/****************************************************************
 *  ReceiverAP + On-board “Vest” Sensor  (v2, Gafas-ready)
 *  - SoftAP  : ESPP_Receiver  (channel 1, WPA2 “12345678”)
 *  - ESP-NOW : listens to all sensors (Clock, GSR, Gafas, Pecho…)
 *  - HTTP    : /devices  ⇒ JSON table
 *  - Vest    : on-board MPU-6050 @ SDA13/SCL14 + flex on GPIO16
 ****************************************************************/
#include <WiFi.h>
#include <esp_now.h>
#include <WebServer.h>
#include <Wire.h>
#include <math.h>

/* ---------- CONFIG -------------------------------------------------- */
static const char*  AP_SSID = "ESPP_Receiver";
static const char*  AP_PSK  = "12345678";
static constexpr int HTTP_PORT   = 80;
static constexpr int MAX_DEVICES = 20;

/* ---------- ESB-P frame --------------------------------------------- */
struct __attribute__((packed)) espMessage {
  char    deviceId[16];
  char    broadcastType[16];
  uint8_t payload[56];
};

/* ---------- Per-device table ---------------------------------------- */
struct Device {
  String  id;
  bool    hasForm=false;   String formJson;
  bool    hasHr  =false;   int    lastHr=0;  unsigned long hrTs=0;
  bool    hasV   =false;   float  lastV=0;                    // GSR
  bool    hasAccel=false;  int16_t ax=0,ay=0,az=0;
  bool    hasOrient=false; float  roll=0, pitch=0;            // derived
  bool    hasFlex =false;  uint16_t flexRes=0;
};
static Device devices[MAX_DEVICES];
static int    deviceCount=0;

/* ---------- HTTP server --------------------------------------------- */
WebServer server(HTTP_PORT);

/* ---------- helpers ------------------------------------------------- */
int findOrAddDevice(const char* id){
  for(int i=0;i<deviceCount;i++)
    if(devices[i].id==id) return i;
  if(deviceCount>=MAX_DEVICES) return -1;
  devices[deviceCount].id=id;
  return deviceCount++;
}

/* ---------- ESP-NOW receive callback -------------------------------- */
void onDataRecv(const uint8_t*,const uint8_t* data,int len){
  if(len!=sizeof(espMessage)) return;
  auto* m = (espMessage*)data;
  int idx = findOrAddDevice(m->deviceId);
  if(idx<0) return;
  Device &d = devices[idx];

  if(!strcmp(m->broadcastType,"FORM")){
    d.hasForm=true;
    d.formJson = String("{")+
      "\"edad\":\""+String((char*)m->payload+0 )+"\","+
      "\"altura\":\""+String((char*)m->payload+4 )+"\","+
      "\"peso\":\""+String((char*)m->payload+8 )+"\","+
      "\"vasos\":\""+String((char*)m->payload+12)+"\","+
      "\"hrs\":\""+String((char*)m->payload+16)+"\","+
      "\"nivel\":\""+String((char*)m->payload+20)+"\"}";
  }
  else if(!strcmp(m->broadcastType,"HR")){
    d.hasHr=true;
    d.lastHr=*reinterpret_cast<const int32_t*>(&m->payload[0]);
    d.hrTs = millis();
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
    /* optional orientation */
    float ax=d.ax/16384.0f, ay=d.ay/16384.0f, az=d.az/16384.0f;
    d.roll  = atan2f( ay, az ) * 57.2958f;
    d.pitch = atan2f(-ax, sqrtf(ay*ay + az*az)) * 57.2958f;
    d.hasOrient = true;
  }
  else if(!strcmp(m->broadcastType,"FLEX")){
    d.hasFlex=true;
    d.flexRes=*reinterpret_cast<const uint16_t*>(&m->payload[0]);
  }
}

/* ---------- /devices ------------------------------------------------ */
String accelJson(const Device& d){
  String s="{\"ax\":"+String(d.ax)+",\"ay\":"+String(d.ay)+",\"az\":"+String(d.az)+"}";
  if(d.hasOrient){
    s.remove(s.length()-1);          // strip '}'
    s += ",\"roll\":"+String(d.roll,1)+",\"pitch\":"+String(d.pitch,1)+"}";
  }
  return s;
}
void handleDevices(){
  String j="{\"devices\":[";
  for(int i=0;i<deviceCount;i++){
    const Device& d=devices[i];
    j+="{\"id\":\""+d.id+"\"";
    if(d.hasForm)   j+=",\"FORM\":"+d.formJson;
    if(d.hasHr)     j+=",\"HR\":"+String(d.lastHr);
    if(d.hasV)      j+=",\"VMEDIDO\":"+String(d.lastV,3);
    if(d.hasAccel)  j+=",\"ACCEL\":"+accelJson(d);
    if(d.hasFlex)   j+=",\"FLEX\":"+String(d.flexRes);
    j+="}";
    if(i<deviceCount-1) j+=",";
  }
  j+="]}";
  server.send(200,"application/json",j);
}

/* ---------- On-board Vest sensor  ----------------------------------- */
#define SDA_V 13
#define SCL_V 14
#define MPU_V 0x68
#define FLEX_PIN 16
const float VREF=3.3; const int ADCMAX=4095; const float RPULL=10000;

void initVest(){
  Wire.begin(SDA_V,SCL_V);
  Wire.beginTransmission(MPU_V);
  Wire.write(0x6B); Wire.write(0); Wire.endTransmission();
  analogReadResolution(12); analogSetAttenuation(ADC_11db);
}
void sampleVest(){
  Wire.beginTransmission(MPU_V); Wire.write(0x3B);
  if(Wire.endTransmission(false)!=0) return;
  Wire.requestFrom(MPU_V,6,true);
  if(Wire.available()!=6) return;
  int16_t ax=(Wire.read()<<8)|Wire.read();
  int16_t ay=(Wire.read()<<8)|Wire.read();
  int16_t az=(Wire.read()<<8)|Wire.read();

  int raw = analogRead(FLEX_PIN);
  float v  = raw*VREF/ADCMAX;
  float r  = (v*RPULL)/(VREF-v);

  int idx=findOrAddDevice("Vest"); if(idx<0) return;
  Device &d=devices[idx];
  d.hasAccel=true; d.ax=ax; d.ay=ay; d.az=az;
  d.hasFlex =true; d.flexRes=(uint16_t)r;
}

/* ---------- Setup --------------------------------------------------- */
void setup(){
  Serial.begin(115200);
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(AP_SSID,AP_PSK,1);
  Serial.printf("SoftAP %s  IP %s\n",AP_SSID,WiFi.softAPIP().toString().c_str());

  esp_now_init();
  esp_now_register_recv_cb(onDataRecv);
  esp_now_peer_info_t p={}; memset(p.peer_addr,0xFF,6); p.channel=1;
  esp_now_add_peer(&p);

  server.on("/devices",handleDevices); server.begin();
  initVest();
}

/* ---------- Loop ---------------------------------------------------- */
unsigned long lastVest=0;
void loop(){
  server.handleClient();
  if(millis()-lastVest>=200){ sampleVest(); lastVest=millis(); }
}
