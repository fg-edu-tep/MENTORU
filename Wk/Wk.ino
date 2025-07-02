/********************************************************************
 *  ClockDevice  —  Round LCD + MAX30102 + ESP-NOW (ESB-P) v1.1
 *
 *  • Two I²C buses
 *      – Wire   : touch CST816S (GPIO6/7/13/5)
 *      – I2C_MAX: MAX30102 HR sensor (GPIO17 SDA, 18 SCL)
 *  • Form wizard  →   IMC (BMI) screen (orange)  →  HR screen
 *  • Broadcasts
 *      – "FORM" once after Enviar
 *      – "HR"  every 5 s with **real** BPM
 ********************************************************************/
#include <Wire.h>
#include <lvgl.h>
#include <TFT_eSPI.h>
#include "CST816S.h"
#include <WiFi.h>
#include <esp_now.h>
#include "MAX30105.h"
#include "heartRate.h"

/* ───────── LVGL / HR timing ───────── */
#define LVGL_TICK_MS    2
#define HR_TX_MS        5000   // 5 s broadcast

/* ───────── Screen & buffer ───────── */
static const uint16_t SW = 240, SH = 240;
static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf[SW*SH/10];

/* ───────── Hardware objects ───────── */
TFT_eSPI   tft(SW, SH);
CST816S    touch(6, 7, 13, 5);
TwoWire    I2C_MAX(1);                 // I²C1  for MAX30102
MAX30105   heart;

/* ───────── ESP-NOW peer ───────── */
const uint8_t BC_ADDR[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

/* ───────── ESB-P frame ───────── */
typedef struct {
  char deviceId[16];
  char broadcastType[16];
  uint8_t payload[56];                 // first 4 = hr for "HR"
} espMessage;
static espMessage frame;

/* ───────── Form state ───────── */
static const char *labels[] = {
  "Edad", "Altura (cm)", "Peso (kg)",
  "# Vasos Agua", "Hrs Dormidas", "Actividad Física"
};
enum { F_EDAD, F_ALT, F_PESO, F_VASOS, F_HRS, F_NIVEL, F_BMI, F_DONE };
static int    step      = 0;           // wizard step
static lv_obj_t *roller,*lblTitle,*lblBpm,*lblAlert,*btnPrev,*btnNext,*lblBMI;

/* Stored answers (ASCII) */
char sEdad[4]="", sAlt[4]="", sPeso[4]="", sVasos[4]="",
     sHrs[4]="", sNivel[16]="";

/* HR processing */
static unsigned long lastBeatMs=0, lastHrSend=0;
static int  bpmCurrent = 0;            // latest measured

/* ───────── Helpers ───────── */
String rangeStr(int lo,int hi){
  String s; for(int v=lo;v<=hi;v++){ s+=String(v); if(v<hi)s+="\n"; }
  return s;
}
float calcBMI(){
  int h=atoi(sAlt), w=atoi(sPeso);
  if(!h) return 0;
  float hm=h/100.0;
  return w/(hm*hm);
}

/* ───────── ESP-NOW init ───────── */
void initEspNow(){
  WiFi.mode(WIFI_STA);
  esp_now_init();
  esp_now_peer_info_t p={};
  memcpy(p.peer_addr,BC_ADDR,6);
  p.channel=1; p.encrypt=false;
  esp_now_add_peer(&p);
}

/* ───────── LVGL display flush ───────── */
void flush_cb(lv_disp_drv_t *d,const lv_area_t *a,lv_color_t *c){
  uint32_t w=a->x2-a->x1+1, h=a->y2-a->y1+1;
  tft.startWrite();
  tft.setAddrWindow(a->x1,a->y1,w,h);
  tft.pushColors((uint16_t*)c,w*h,true);
  tft.endWrite();
  lv_disp_flush_ready(d);
}
void lv_tick(void*){ lv_tick_inc(LVGL_TICK_MS); }
void read_touch(lv_indev_drv_t*,lv_indev_data_t *d){
  if(!touch.available()) d->state=LV_INDEV_STATE_REL;
  else{
    d->state=LV_INDEV_STATE_PR;
    d->point.x=touch.data.x; d->point.y=touch.data.y;
  }
}

/* ───────── Wizard UI builders ───────── */
void buildForm(){
  lv_obj_t *scr=lv_scr_act(); lv_obj_clean(scr);

  lblTitle = lv_label_create(scr);
  lv_label_set_text(lblTitle, labels[step]);
  lv_obj_align(lblTitle, LV_ALIGN_TOP_MID, 0, 20);

  roller = lv_roller_create(scr);
  lv_obj_set_size(roller, 180, 50);
  lv_obj_align(roller, LV_ALIGN_CENTER, 0, 0);
  lv_roller_set_visible_row_count(roller,3);

  switch(step){
    case F_EDAD  : lv_roller_set_options(roller,rangeStr(0,120).c_str(),LV_ROLLER_MODE_NORMAL); break;
    case F_ALT   : lv_roller_set_options(roller,rangeStr(100,220).c_str(),LV_ROLLER_MODE_NORMAL); break;
    case F_PESO  : lv_roller_set_options(roller,rangeStr(30,200).c_str(),LV_ROLLER_MODE_NORMAL); break;
    case F_VASOS : lv_roller_set_options(roller,rangeStr(0,20).c_str(),LV_ROLLER_MODE_NORMAL); break;
    case F_HRS   : lv_roller_set_options(roller,rangeStr(0,24).c_str(),LV_ROLLER_MODE_NORMAL); break;
    case F_NIVEL : lv_roller_set_options(roller,"Bajo\nModerado\nAlto\nMuy Alto",LV_ROLLER_MODE_NORMAL); break;
  }

  btnPrev = lv_btn_create(scr);
  lv_obj_set_size(btnPrev,60,40);
  lv_obj_align(btnPrev, LV_ALIGN_BOTTOM_LEFT,10,-10);
  lv_label_set_text(lv_label_create(btnPrev),"<");
  lv_obj_add_event_cb(btnPrev,[](lv_event_t*,void*){ if(step>0){--step; buildForm(); } },LV_EVENT_CLICKED,nullptr);

  btnNext = lv_btn_create(scr);
  lv_obj_set_size(btnNext,80,40);
  lv_obj_align(btnNext, LV_ALIGN_BOTTOM_RIGHT,-10,-10);
  lv_label_set_text(lv_label_create(btnNext), (step==F_NIVEL?"Enviar":">"));
  lv_obj_add_event_cb(btnNext,[](lv_event_t*,void*){
        char sel[16]; lv_roller_get_selected_str(roller,sel,sizeof(sel));
        switch(step){
          case F_EDAD : strcpy(sEdad ,sel); break;
          case F_ALT  : strcpy(sAlt  ,sel); break;
          case F_PESO : strcpy(sPeso ,sel); lastBMI=calcBMI(); break;
          case F_VASOS: strcpy(sVasos,sel); break;
          case F_HRS  : strcpy(sHrs  ,sel); break;
          case F_NIVEL:strcpy(sNivel,sel); break;
        }
        if(step==F_PESO){ step=F_BMI; buildBmi(); return; }
        if(step<F_NIVEL){ ++step; buildForm(); return; }

        /* SEND FORM */
        memset(&frame,0,sizeof(frame));
        strncpy(frame.deviceId,"Clock",15);
        strncpy(frame.broadcastType,"FORM",15);
        strncpy((char*)frame.payload+0 ,sEdad ,3);
        strncpy((char*)frame.payload+4 ,sAlt  ,3);
        strncpy((char*)frame.payload+8 ,sPeso ,3);
        strncpy((char*)frame.payload+12,sVasos,3);
        strncpy((char*)frame.payload+16,sHrs  ,3);
        strncpy((char*)frame.payload+20,sNivel,15);
        esp_now_send(BC_ADDR,(uint8_t*)&frame,sizeof(frame));
        step = F_DONE;
        buildHeart();
  },LV_EVENT_CLICKED,nullptr);
}

void buildBmi(){
  lv_obj_t *scr=lv_scr_act(); lv_obj_clean(scr);
  lv_obj_set_style_bg_color(scr, lv_palette_main(LV_PALETTE_ORANGE), 0);
  lblBMI = lv_label_create(scr);
  char buf[32]; snprintf(buf,sizeof(buf),"Tu IMC es:\n%.1f", lastBMI);
  lv_label_set_text(lblBMI,buf);
  lv_obj_center(lblBMI);

  lv_obj_t *btn = lv_btn_create(scr);
  lv_obj_set_size(btn,80,40);
  lv_obj_align(btn, LV_ALIGN_BOTTOM_RIGHT,-10,-10);
  lv_label_set_text(lv_label_create(btn),">");
  lv_obj_add_event_cb(btn,[](lv_event_t*,void*){ step=F_VASOS; buildForm(); },LV_EVENT_CLICKED,nullptr);
}

void buildHeart(){
  lv_obj_t *scr=lv_scr_act(); lv_obj_clean(scr);

  lv_obj_t *c=lv_obj_create(scr);
  lv_obj_set_size(c,120,120);
  lv_obj_set_style_radius(c,LV_RADIUS_CIRCLE,0);
  lv_obj_align(c,LV_ALIGN_TOP_MID,0,10);

  lblBpm = lv_label_create(c); lv_label_set_text(lblBpm,"BPM:\n--");
  lv_obj_center(lblBpm);

  lblAlert = lv_label_create(scr);
  lv_label_set_text(lblAlert,"Alerts: None");
  lv_obj_align(lblAlert,LV_ALIGN_BOTTOM_MID,0,-10);

  btnPrev = lv_btn_create(scr);
  lv_obj_set_size(btnPrev,60,40);
  lv_obj_align(btnPrev,LV_ALIGN_BOTTOM_LEFT,10,-10);
  lv_label_set_text(lv_label_create(btnPrev),"<");
  lv_obj_add_event_cb(btnPrev,[](lv_event_t*,void*){ step=0; buildForm(); },LV_EVENT_CLICKED,nullptr);
}

/* ───────── setup ───────── */
void setup(){
  Serial.begin(115200);

  /* Display & touch */
  lv_init(); tft.begin(); tft.setRotation(0);
  touch.begin();

  lv_disp_draw_buf_init(&draw_buf,buf,NULL,SW*SH/10);
  static lv_disp_drv_t dd; lv_disp_drv_init(&dd);
  dd.hor_res=SW; dd.ver_res=SH; dd.flush_cb=flush_cb; dd.draw_buf=&draw_buf;
  lv_disp_drv_register(&dd);
  static lv_indev_drv_t id; lv_indev_drv_init(&id);
  id.type=LV_INDEV_TYPE_POINTER; id.read_cb=read_touch;
  lv_indev_drv_register(&id);
  static esp_timer_create_args_t ta={.callback=&lv_tick};
  esp_timer_handle_t tm; esp_timer_create(&ta,&tm);
  esp_timer_start_periodic(tm,LVGL_TICK_MS*1000);

  /* MAX30102 */
  I2C_MAX.begin(17,18,400000);
  if(!heart.begin(I2C_MAX)) Serial.println("MAX30102 not found!");
  heart.setup(0x1F,4,2,100,411,4096);

  initEspNow();
  buildForm();
}

/* ───────── loop ───────── */
void loop(){
  lv_timer_handler();

  /* Heart-rate sensor */
  long ir = heart.getIR();
  if( checkForBeat(ir) ){
    unsigned long now=millis();
    unsigned long dt = now-lastBeatMs; lastBeatMs=now;
    if(dt>300){
      bpmCurrent = int(60000.0/dt + 0.5);
      char b[16]; snprintf(b,sizeof(b),"BPM:\n%d",bpmCurrent);
      if(lblBpm) lv_label_set_text(lblBpm,b);
      Serial.printf("RAW IR:%ld  BPM:%d\n",ir,bpmCurrent);
    }
  }

  /* transmit BPM every HR_TX_MS */
  if( millis()-lastHrSend >= HR_TX_MS && bpmCurrent>0 && step==F_DONE ){
    memset(&frame,0,sizeof(frame));
    strncpy(frame.deviceId,"Clock",15);
    strncpy(frame.broadcastType,"HR",15);
    memcpy(frame.payload,&bpmCurrent,sizeof(int));
    esp_now_send(BC_ADDR,(uint8_t*)&frame,sizeof(frame));
    lastHrSend=millis();
  }
}
