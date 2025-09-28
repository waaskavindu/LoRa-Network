#include <Arduino.h>
#include "LoRaWan_APP.h"
#include <Wire.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include "HT_SSD1306Wire.h"
#include "HT_DisplayUi.h"
#include <ctype.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

// ---------- Forward decls (avoid Arduino auto-prototype gotchas) ----------
struct Telemetry;
static bool influxWriteLP(const Telemetry& m);
void taskInflux(void*);

// ========================= OLED (Seq/T/H/P only) =========================
#ifdef WIRELESS_STICK_V3
static SSD1306Wire display(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_64_32, RST_OLED);
#else
static SSD1306Wire display(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED);
#endif

static void VextON(){ pinMode(Vext, OUTPUT); digitalWrite(Vext, LOW); }
static void VextOFF(){ pinMode(Vext, OUTPUT); digitalWrite(Vext, HIGH); }

static inline void drawLabel(int x,int y,const String&s){
  display.setColor(WHITE);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_10);
  display.drawString(x,y,s);
}
static void drawVal(int x,int y,int w,int h,const String& s){
  display.setColor(BLACK); display.fillRect(x,y,w,h);
  display.setColor(WHITE); display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_10); display.drawString(x,y,s);
}

// Regions (128x64; fine on 64x32 with tighter spacing)
const int R_SEQ[4] = {34, 0,  90, 10};
const int R_T[4]   = {14, 12, 40, 10};
const int R_H[4]   = {74, 12, 40, 10};
const int R_P[4]   = {14, 24, 60, 10};

// ========================= Wi-Fi (EDIT) =========================
const char* WIFI_SSID     = "kavinduS24";
const char* WIFI_PASSWORD = "12345678";
static void wifiEnsure() {
  if (WiFi.status()==WL_CONNECTED) return;
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

// ========================= Influx v2 (EDIT) =========================
const char* INFLUX_URL   ="http://192.168.72.217:8086/api/v2/write?orgID=de71bdc994f4c039&bucket=lora&precision=ns";
const char* INFLUX_TOKEN ="wzml4XRuPMpn94jmmTaObiaOcfqpY2BMPA4Wklmf131LsEbvCQaqghm2fM961sZ3eBbxz8ZVEyRAOdyJanzgFg==";
const char* DEVICE_ID    ="heltec-v3-1";

// Telemetry for Influx
struct Telemetry {
  int seq;                 // -1 if missing (then we won't write it)
  bool hasT, hasH, hasP;
  float t,h,p;
  int rssi, snr;
};

static QueueHandle_t qInflux;

static bool influxWriteLP(const Telemetry& m){
  // Build fields; ONLY include seq if >=0
  String fields;
  bool first = true;
  auto addF = [&](const String& k, const String& v){
    if (!first) fields += ",";
    fields += k + "=" + v;
    first = false;
  };
  if (m.seq >= 0) addF("seq",  String(m.seq) + "i");
  if (m.hasT)     addF("temp", String(m.t, 2));
  if (m.hasH)     addF("hum",  String(m.h, 2));
  if (m.hasP)     addF("pres", String(m.p, 2));
  addF("rssi", String(m.rssi) + "i");
  addF("snr",  String(m.snr)  + "i");

  String lp = "lora_rx,device=" + String(DEVICE_ID) + " " + fields;
  // IMPORTANT: do NOT append a timestamp → Influx assigns server time

  HTTPClient http; http.setTimeout(800);
  http.begin(INFLUX_URL);
  http.addHeader("Authorization", String("Token ")+INFLUX_TOKEN);
  http.addHeader("Content-Type", "text/plain; charset=utf-8");
  int code = http.POST(lp);
  http.end();

  Serial.printf("[HTTP] %d  LP: %s\n", code, lp.c_str());
  return code>=200 && code<300;
}

void taskInflux(void*){
  WiFi.setSleep(false);
  for(;;){
    Telemetry m;
    if (xQueueReceive(qInflux, &m, pdMS_TO_TICKS(200))){
      if (WiFi.status()!=WL_CONNECTED) wifiEnsure();
      (void)influxWriteLP(m);
    }
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

// ========================= LoRa PHY =========================
#define RF_FREQUENCY        915000000UL
#define LORA_BANDWIDTH      0      // 125 kHz (Heltec: 0=125k,1=250k,2=500k)
#define LORA_SPREADING      7
#define LORA_CODINGRATE     1
#define LORA_PREAMBLE_LEN   8
#define LORA_IQ_INVERSION   false

#ifndef SLOW_CLK_TYPE
#define SLOW_CLK_TYPE SLOW_CLK_TPYE
#endif

static RadioEvents_t RadioEvents;

// ========================= Packet handoff =========================
static const uint16_t BUF_SZ = 240;
static volatile bool   packetReady = false;
static char            rxBuf[BUF_SZ];
static volatile int16_t rxRssi = 0;
static volatile int8_t  rxSnr  = 0;
static uint32_t lastRxMs = 0;

void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) {
  uint16_t n = size < (BUF_SZ-1) ? size : (BUF_SZ-1);
  memcpy(rxBuf, payload, n); rxBuf[n] = '\0';
  rxRssi = rssi; rxSnr = snr;
  packetReady = true; lastRxMs = millis();
  Radio.Standby();
}

// ========================= Robust parsing =========================
static inline void trim(char*& s){ while (*s==' '||*s=='\t') s++; }

static bool parseNumber(const char* s, float& out){
  while (*s==' '||*s=='\t') s++;
  if (strncasecmp(s,"nan",3)==0) return false;
  char* endp=nullptr; out=strtof(s,&endp);
  return (endp && endp!=s && isfinite(out));
}

// Accept keys: seq/count/cnt/id, temp/t, hum/h/rh/humidity, pres/p/press/pressure
// Accept separators ':', '=', '#'
static void parsePayload(const char* s,
                         int& seq, bool& hasT, float& t,
                         bool& hasH, float& h,
                         bool& hasP, float& p)
{
  seq=-1; hasT=hasH=hasP=false; t=h=p=NAN;
  char buf[BUF_SZ]; strncpy(buf,s,sizeof(buf)-1); buf[sizeof(buf)-1]='\0';

  for (char* tok=strtok(buf,","); tok; tok=strtok(nullptr,",")){
    trim(tok);
    char* sep = strpbrk(tok, ":=#");
    if (!sep) continue;
    *sep = '\0';
    char* key = tok;
    char* val = sep+1;
    trim(key); trim(val);

    for (char* c=key; *c; ++c) *c = tolower(*c);

    if (!strcmp(key,"count") || !strcmp(key,"cnt") || !strcmp(key,"seq") || !strcmp(key,"id")){
      seq = atoi(val);
      continue;
    }
    if (!strcmp(key,"temp") || !strcmp(key,"t")){
      float v; if (parseNumber(val, v)) { t=v; hasT=true; } continue;
    }
    if (!strcmp(key,"hum") || !strcmp(key,"h") || !strcmp(key,"rh") || !strcmp(key,"humidity")){
      float v; if (parseNumber(val, v)) { h=v; hasH=true; } continue;
    }
    if (!strcmp(key,"pres") || !strcmp(key,"p") || !strcmp(key,"press") || !strcmp(key,"pressure")){
      float v; if (parseNumber(val, v)) { p=v; hasP=true; } continue;
    }
  }
}

// ========================= OLED cache =========================
String lastSeq="", lastT="", lastH="", lastP="";
volatile bool dirtyUI = false;

// ========================= Setup =========================
void setup() {
  Serial.begin(115200);

  VextON(); delay(20);
  Wire.begin(SDA_OLED, SCL_OLED);
  display.init(); display.displayOn(); display.setContrast(255);

  display.clear();
  drawLabel(0,  0, "Seq:");
  drawLabel(0, 12, "T:");
  drawLabel(60,12, "H:");
  drawLabel(0, 24, "P:");
  display.display();

  wifiEnsure();

  qInflux = xQueueCreate(64, sizeof(Telemetry));
  xTaskCreatePinnedToCore(taskInflux, "taskInflux", 4096, nullptr, 1, nullptr, 0);

  Mcu.begin(HELTEC_BOARD, SLOW_CLK_TYPE);
  RadioEvents.RxDone = OnRxDone;
  Radio.Init(&RadioEvents);
  Radio.SetChannel(RF_FREQUENCY);
  Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING, LORA_CODINGRATE,
                    0, LORA_PREAMBLE_LEN, 0, false, 0, true, 0, 0,
                    LORA_IQ_INVERSION, true);

  Radio.Rx(0);
  lastRxMs = millis();
  Serial.println("[RX] Ready");
}

// ========================= Loop (non-blocking) =========================
void loop() {
  Radio.IrqProcess();

  // light Wi-Fi upkeep
  static uint32_t lastNet=0;
  uint32_t now = millis();
  if (now - lastNet > 2000){
    if (WiFi.status()!=WL_CONNECTED) wifiEnsure();
    lastNet = now;
  }

  // RX watchdog
  if (now - lastRxMs > 15000){
    Radio.Standby(); Radio.SetChannel(RF_FREQUENCY); Radio.Rx(0);
    lastRxMs = now;
  }

  if (packetReady){
    packetReady = false;

    int seq; bool hasT, hasH, hasP; float t,h,p;
    parsePayload(rxBuf, seq, hasT, t, hasH, h, hasP, p);

    // OLED
    String sSeq = (seq>=0)? String(seq) : "--";
    String sT   = hasT? String(t,1) : "--";
    String sH   = hasH? String(h,1) : "--";
    String sP   = hasP? String(p,1) : "--";

    if (sSeq != lastSeq){ drawVal(R_SEQ[0], R_SEQ[1], R_SEQ[2], R_SEQ[3], sSeq); lastSeq=sSeq; dirtyUI=true; }
    if (sT   != lastT  ){ drawVal(R_T[0],   R_T[1],   R_T[2],   R_T[3],   sT  ); lastT=sT;   dirtyUI=true; }
    if (sH   != lastH  ){ drawVal(R_H[0],   R_H[1],   R_H[2],   R_H[3],   sH  ); lastH=sH;   dirtyUI=true; }
    if (sP   != lastP  ){ drawVal(R_P[0],   R_P[1],   R_P[2],   R_P[3],   sP  ); lastP=sP;   dirtyUI=true; }

    // enqueue to Influx (note: seq omitted if missing)
    Telemetry m;
    m.seq   = seq;
    m.hasT  = hasT;  m.t = t;
    m.hasH  = hasH;  m.h = h;
    m.hasP  = hasP;  m.p = p;
    m.rssi  = (int)rxRssi;
    m.snr   = (int)rxSnr;
    xQueueSend(qInflux, &m, pdMS_TO_TICKS(50));

    Radio.Rx(0);
    lastRxMs = now;
  }

  // OLED flush ~15 Hz
  static uint32_t lastFlush = 0;
  if (dirtyUI && (now - lastFlush >= 66)){
    display.display();
    dirtyUI = false;
    lastFlush = now;
  }

  vTaskDelay(1);
}
