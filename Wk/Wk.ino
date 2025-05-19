/* ClockDevice.ino
 * – Round-display form + heart UI with LVGL
 * – ESPNOW messages tagged with deviceId="Clock"
 *   and broadcastType="FORM" or "HR"
 * – Periodic HR broadcast every 30 s
 */

#include <lvgl.h>
#include <TFT_eSPI.h>
#include "CST816S.h"
#include <WiFi.h>
#include <esp_now.h>

#define LVGL_TICK_PERIOD_MS 2
#define HR_PERIOD_MS      5000  // 30 s

static const uint16_t SCREEN_W = 240, SCREEN_H = 240;

// Draw buffer
static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf[SCREEN_W*SCREEN_H/10];

// Display & touch
TFT_eSPI tft(SCREEN_W, SCREEN_H);
CST816S touch(6, 7, 13, 5);

// ESPNOW broadcast address (all FF)
static uint8_t broadcastAddress[] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

// —————————————————————————————————————————————————————————————————
// Message struct
typedef struct {
  char deviceId[16];       // e.g. "Clock"
  char broadcastType[8];   // "FORM" or "HR"
  // FORM payload:
  char edad[4], altura[4], peso[4];
  char vasos[4], hrs[4], nivel[16];
  // HR payload:
  int  hr;
} espMessage;
// —————————————————————————————————————————————————————————————————

static espMessage msg;       // will fill & send
static unsigned lastHrMs = 0;

// LVGL objects and state for form/heart pages
static int    currentField = 0;
static lv_obj_t *labelField, *rollerField, *btnPrev, *btnNext;
static lv_obj_t *labelBPM, *labelAlert;

const int numFields = 6;
const char *fieldLabels[numFields] = {
  "Edad", "Altura (cm)", "Peso (kg)",
  "# Vasos Agua", "Hrs Dormidas", "Actividad Física"
};

String makeRange(int lo,int hi){
  String s;
  for(int v=lo; v<=hi; ++v){
    s += String(v);
    if(v<hi) s += "\n";
  }
  return s;
}

// —————————————————————————————————————————————————————————————————
// ESPNOW setup
void setupESPNOW(){
  WiFi.mode(WIFI_STA);
  esp_now_init();
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, broadcastAddress, 6);
  peer.channel = 1;
  peer.encrypt = false;
  esp_now_add_peer(&peer);
}

// —————————————————————————————————————————————————————————————————
// LVGL flush & touch
void disp_flush(lv_disp_drv_t *d, const lv_area_t *a, lv_color_t *c){
  uint32_t w=a->x2-a->x1+1, h=a->y2-a->y1+1;
  tft.startWrite();
  tft.setAddrWindow(a->x1,a->y1,w,h);
  tft.pushColors((uint16_t*)c,w*h,true);
  tft.endWrite();
  lv_disp_flush_ready(d);
}
void lv_tick_cb(void*){ lv_tick_inc(LVGL_TICK_PERIOD_MS); }
void touch_read(lv_indev_drv_t*, lv_indev_data_t *d){
  if(!touch.available()) d->state=LV_INDEV_STATE_REL;
  else{
    d->state=LV_INDEV_STATE_PR;
    d->point.x=touch.data.x;
    d->point.y=touch.data.y;
  }
}

// —————————————————————————————————————————————————————————————————
// Navigation
void showFormPage();
void showHeartPage();

void onPrev(lv_event_t*){
  if(currentField>0){
    currentField--;
    showFormPage();
  } else if(currentField>=numFields){
    currentField = numFields-1;
    showFormPage();
  }
}

void onNext(lv_event_t*){
  // read roller into msg when on last field
  char bufv[16];
  lv_roller_get_selected_str(rollerField, bufv, sizeof(bufv));
  switch(currentField){
    case 0: strcpy(msg.edad, bufv); break;
    case 1: strcpy(msg.altura, bufv); break;
    case 2: strcpy(msg.peso, bufv); break;
    case 3: strcpy(msg.vasos, bufv); break;
    case 4: strcpy(msg.hrs, bufv); break;
    case 5: strcpy(msg.nivel, bufv); break;
  }
  if(currentField < numFields-1){
    currentField++;
    showFormPage();
  } else {
    // Submit FORM
    strcpy(msg.deviceId,    "Clock");
    strcpy(msg.broadcastType, "FORM");
    // hr field unused here
    esp_now_send(broadcastAddress, (uint8_t*)&msg, sizeof(msg));
    showHeartPage();
    lastHrMs = millis();
  }
}

// —————————————————————————————————————————————————————————————————
// Build form page
void showFormPage(){
  lv_obj_t *scr = lv_scr_act();
  lv_obj_clean(scr);

  // title
  labelField = lv_label_create(scr);
  lv_label_set_text(labelField, fieldLabels[currentField]);
  lv_obj_align(labelField, LV_ALIGN_TOP_MID, 0, 20);

  // roller
  rollerField = lv_roller_create(scr);
  lv_obj_set_size(rollerField, 180, 50);
  lv_obj_align(rollerField, LV_ALIGN_CENTER, 0, 0);
  lv_roller_set_visible_row_count(rollerField,3);

  String opts;
  switch(currentField){
    case 0: opts=makeRange(0,120); break;
    case 1: opts=makeRange(100,220); break;
    case 2: opts=makeRange(30,200); break;
    case 3: opts=makeRange(0,20); break;
    case 4: opts=makeRange(0,24); break;
    case 5: opts="Bajo\nModerado\nAlto\nMuy Alto"; break;
  }
  lv_roller_set_options(rollerField, opts.c_str(), LV_ROLLER_MODE_NORMAL);

  // Prev
  btnPrev = lv_btn_create(scr);
  lv_obj_set_size(btnPrev,60,40);
  lv_obj_align(btnPrev, LV_ALIGN_BOTTOM_LEFT,10,-10);
  lv_obj_add_event_cb(btnPrev, onPrev, LV_EVENT_CLICKED, nullptr);
  lv_label_set_text(lv_label_create(btnPrev), "<");

  // Next/Enviar
  btnNext = lv_btn_create(scr);
  lv_obj_set_size(btnNext,80,40);
  lv_obj_align(btnNext, LV_ALIGN_BOTTOM_RIGHT,-10,-10);
  lv_obj_add_event_cb(btnNext, onNext, LV_EVENT_CLICKED, nullptr);
  lv_label_set_text(lv_label_create(btnNext),
    currentField < numFields-1 ? ">" : "Enviar");
}

// —————————————————————————————————————————————————————————————————
// Build heart-rate page
void showHeartPage(){
  currentField = numFields;
  lv_obj_t *scr = lv_scr_act();
  lv_obj_clean(scr);

  // BPM circle
  lv_obj_t *c = lv_obj_create(scr);
  lv_obj_set_size(c,120,120);
  lv_obj_align(c,LV_ALIGN_TOP_MID,0,10);
  lv_obj_set_style_radius(c,LV_RADIUS_CIRCLE,0);
  labelBPM = lv_label_create(c);
  lv_label_set_text(labelBPM,"BPM: --");
  lv_obj_center(labelBPM);

  // Alerts
  labelAlert = lv_label_create(scr);
  lv_label_set_text(labelAlert,"Alerts: None");
  lv_obj_align(labelAlert, LV_ALIGN_BOTTOM_MID,0,-10);

  // Back
  btnPrev = lv_btn_create(scr);
  lv_obj_set_size(btnPrev,60,40);
  lv_obj_align(btnPrev,LV_ALIGN_BOTTOM_LEFT,10,-10);
  lv_obj_add_event_cb(btnPrev, onPrev, LV_EVENT_CLICKED,nullptr);
  lv_label_set_text(lv_label_create(btnPrev),"<");
}

// —————————————————————————————————————————————————————————————————
// setup & loop
void setup(){
  Serial.begin(115200);
  randomSeed(analogRead(0));

  setupESPNOW();

  // LVGL init
  lv_init();
  tft.begin(); tft.setRotation(0);
  touch.begin();

  lv_disp_draw_buf_init(&draw_buf, buf, NULL, SCREEN_W*SCREEN_H/10);
  static lv_disp_drv_t dd; lv_disp_drv_init(&dd);
  dd.hor_res=SCREEN_W; dd.ver_res=SCREEN_H;
  dd.flush_cb=disp_flush; dd.draw_buf=&draw_buf;
  lv_disp_drv_register(&dd);

  static lv_indev_drv_t id; lv_indev_drv_init(&id);
  id.type=LV_INDEV_TYPE_POINTER; id.read_cb=touch_read;
  lv_indev_drv_register(&id);

  const esp_timer_create_args_t ta = { .callback = &lv_tick_cb };
  esp_timer_handle_t tm; esp_timer_create(&ta, &tm);
  esp_timer_start_periodic(tm, LVGL_TICK_PERIOD_MS*1000);

  showFormPage();
}

void loop(){
  lv_timer_handler();
  delay(5);

  unsigned long now = millis();
  if(now - lastHrMs >= HR_PERIOD_MS) {
    // fill the HR message
    msg.hr = random(60,100);
    strcpy(msg.deviceId,    "Clock");
    strcpy(msg.broadcastType,"HR");
    esp_now_send(broadcastAddress, (uint8_t*)&msg, sizeof(msg));
    lastHrMs = now;
  }

  // periodic HR broadcast
  if(currentField == numFields){
    unsigned now = millis();
    if(now - lastHrMs >= HR_PERIOD_MS){
      msg.hr = random(60,100);
      strcpy(msg.deviceId,    "Clock");
      strcpy(msg.broadcastType,"HR");
      esp_now_send(broadcastAddress, (uint8_t*)&msg, sizeof(msg));
      lastHrMs = now;
    }
  }
  delay(5);
}
