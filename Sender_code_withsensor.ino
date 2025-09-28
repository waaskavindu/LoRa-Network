#include <Arduino.h>
#include "LoRaWan_APP.h"
#include <Wire.h>

#include "HT_SSD1306Wire.h"
#include "HT_DisplayUi.h"

// Sensor libs
#include <Adafruit_AHTX0.h>
#include <Adafruit_BMP280.h>

// ---------- OLED wiring ----------
#ifdef WIRELESS_STICK_V3
static SSD1306Wire display(0x3c, 500000, SDA_OLED, SCL_OLED,
                           GEOMETRY_64_32, RST_OLED);
#else
static SSD1306Wire display(0x3c, 500000, SDA_OLED, SCL_OLED,
                           GEOMETRY_128_64, RST_OLED);
#endif

static void VextON()  { pinMode(Vext, OUTPUT); digitalWrite(Vext, LOW);  }
static void VextOFF() { pinMode(Vext, OUTPUT); digitalWrite(Vext, HIGH); }

// ---------- LoRa PHY ----------
#define RF_FREQUENCY        915000000UL
#define LORA_BANDWIDTH      0            // 125kHz
#define LORA_SPREADING      7            // SF7
#define LORA_CODINGRATE     1            // 4/5
#define LORA_PREAMBLE_LEN   8
#define LORA_IQ_INVERSION   false
#define TX_POWER_DBM        14

#ifndef SLOW_CLK_TYPE
#define SLOW_CLK_TYPE SLOW_CLK_TPYE
#endif

static RadioEvents_t RadioEvents;
static volatile bool txDone = false;
void OnTxDone(void) { txDone = true; }

// ---------- Sensors on second I2C ----------
TwoWire I2CBus = TwoWire(1);   // Second I2C bus
#define I2C_SDA 20
#define I2C_SCL 19

Adafruit_AHTX0 aht;
Adafruit_BMP280 bmp(&I2CBus);  // attach to custom bus

bool sensorsOK = false;

// ---------- OLED helper ----------
static void oledShow(uint32_t seq, float temp, float hum, float pres) {
  display.clear();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_10);
  display.drawString(0, 0,  "Seq: " + String(seq));
  display.drawString(0, 12, "Temp: " + String(temp, 1) + " C");
  display.drawString(0, 24, "Hum : " + String(hum, 1) + " %");
  display.drawString(0, 36, "Pres: " + String(pres, 1) + " hPa");
  display.display();
}

// ---------- Setup ----------
void setup() {
  Serial.begin(115200);
  delay(20);

  VextON(); delay(30);
  Wire.begin(SDA_OLED, SCL_OLED);   // for OLED
  I2CBus.begin(I2C_SDA, I2C_SCL);   // for sensors

  // OLED init
  display.init();
  display.displayOn();
  display.setContrast(255);
  display.clear();
  display.drawString(0, 0, "Booting...");
  display.display();

  // LoRa init
  Mcu.begin(HELTEC_BOARD, SLOW_CLK_TYPE);
  RadioEvents.TxDone = OnTxDone;
  Radio.Init(&RadioEvents);
  Radio.SetChannel(RF_FREQUENCY);
  Radio.SetTxConfig(MODEM_LORA, TX_POWER_DBM, 0,
                    LORA_BANDWIDTH, LORA_SPREADING, LORA_CODINGRATE,
                    LORA_PREAMBLE_LEN, false, true, 0, 0,
                    LORA_IQ_INVERSION, 3000);

  // Sensors init
  if (!aht.begin(&I2CBus)) {
    Serial.println("AHT20 not found!");
  } else {
    Serial.println("AHT20 OK");
    sensorsOK = true;
  }

  if (!bmp.begin(0x77)) { // Your sensor = 0x77
    Serial.println("BMP280 not found!");
  } else {
    Serial.println("BMP280 OK");
    sensorsOK = true;
  }

  if (sensorsOK) {
    display.drawString(0, 12, "Sensors OK");
  } else {
    display.drawString(0, 12, "No sensors!");
  }
  display.display();
}

// ---------- Loop ----------
void loop() {
  static uint32_t seq = 0;
  char msg[96];

  float temp = NAN, hum = NAN, pres = NAN;

  if (sensorsOK) {
    sensors_event_t humidity, temperature;
    aht.getEvent(&humidity, &temperature);
    temp = temperature.temperature;
    hum  = humidity.relative_humidity;
    pres = bmp.readPressure() / 100.0F;  // hPa
  }

  // Build LoRa message
  snprintf(msg, sizeof(msg),
           "count:%lu,temp:%.2f,hum:%.2f,pres:%.2f",
           (unsigned long)seq, temp, hum, pres);

  // Show values on OLED
  oledShow(seq, temp, hum, pres);

  // Send via LoRa
  txDone = false;
  Radio.Send((uint8_t*)msg, strlen(msg));

  uint32_t t0 = millis();
  while (!txDone && (millis() - t0 < 4000)) {
    Radio.IrqProcess();
    delay(10);
  }

  Serial.printf("[TX] %s (%s)\r\n", msg, txDone ? "done" : "timeout");
  seq++;

  // pacing ~2s
  uint32_t start = millis();
  while (millis() - start < 2000) {
    Radio.IrqProcess();
    delay(10);
  }
}
