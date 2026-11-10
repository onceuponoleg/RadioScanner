#pragma once
#include <Arduino.h>
#include <LovyanGFX.hpp>
#include "config.h"

// ================= ДРАЙВЕР ЭКРАНА 1.9" IPS (320x170) =================
class LGFX : public lgfx::LGFX_Device {
  lgfx::Panel_ST7789 _panel;
  lgfx::Bus_SPI _bus;
public:
  LGFX(void) {
    {
      auto cfg = _bus.config();
      cfg.spi_host = VSPI_HOST;
      cfg.spi_mode = 0;
      cfg.freq_write = 20000000;
      cfg.pin_sclk = TFT_SCLK;
      cfg.pin_mosi = TFT_MOSI;
      cfg.pin_miso = -1;
      cfg.pin_dc   = TFT_DC;
      _bus.config(cfg);
      _panel.setBus(&_bus);
    }
    {
      auto cfg = _panel.config();
      cfg.pin_cs   = TFT_CS;
      cfg.pin_rst  = TFT_RST;
      cfg.pin_busy = -1;
      cfg.panel_width  = 170;
      cfg.panel_height = 320;
      cfg.offset_x     = 35;
      cfg.offset_y     = 0;
      cfg.invert       = true;
      _panel.config(cfg);
    }
    setPanel(&_panel);
  }
};

LGFX tft;
LGFX_Sprite canvas(&tft);

bool hasNRF = false;
uint8_t nrfReg06 = 0;
bool soundEnabled = true;

int currentMode = 0; // 0..5 (отображаются как [1/6]..[6/6])
int subView = 0;

// ================= КВАДРАТУРНЫЙ АВТОМАТ ЭНКОДЕРА (БЕЗ ПРОПУСКОВ) =================
volatile int32_t encDetents = 1000;
volatile uint8_t encState = 0;
volatile int8_t  encSubAccum = 0;
static const int8_t DRAM_ATTR ENC_TABLE[16] = {
   0, -1,  1,  0,
   1,  0,  0, -1,
  -1,  0,  0,  1,
   0,  1, -1,  0
};

void IRAM_ATTR encQuadISR() {
  uint8_t s1 = (uint8_t)digitalRead(ENC_S1);
  uint8_t s2 = (uint8_t)digitalRead(ENC_S2);
  encState = ((encState << 2) | (s1 << 1) | s2) & 0x0F;
  int8_t dir = ENC_TABLE[encState];
  if (dir != 0) {
    encSubAccum += dir;
    if (encSubAccum >= 2)      { encDetents++; encSubAccum = 0; }
    else if (encSubAccum <= -2){ encDetents--; encSubAccum = 0; }
  }
}

// ================= ЗВУКОВОЙ ДВИЖОК (ГЕЙГЕР + ВАРИОМЕТР + СИРЕНА) =================
inline void playClick(uint16_t freq, uint16_t durMs) {
  if (!soundEnabled) return;
  tone(BUZZ_PIN, freq, durMs);
}

inline void geigerTick(int intensity) {
  if (!soundEnabled || intensity < 15) return;
  static uint32_t lastTickMs = 0;
  uint32_t minGap = map(constrain(intensity, 15, 255), 15, 255, 180, 12);
  if (millis() - lastTickMs >= minGap) {
    if (random(0, 260) < intensity + 40) {
      uint16_t pitch = map(constrain(intensity, 15, 255), 15, 255, 2100, 4300);
      tone(BUZZ_PIN, pitch, 4);
      lastTickMs = millis();
    }
  }
}

// Звуковой вариометр для режима Пеленгатора (Fox-Hunt): меняет частоту и пульс от RSSI
inline void variometerBeep(int rssiDbm, int thresholdDbm) {
  if (!soundEnabled || rssiDbm < thresholdDbm) return;
  static uint32_t lastVarioMs = 0;
  int clamped = constrain(rssiDbm, -95, -25);
  uint32_t periodMs = map(clamped, -95, -25, 380, 45);
  if (millis() - lastVarioMs >= periodMs) {
    uint16_t pitch = map(clamped, -95, -25, 600, 3400);
    tone(BUZZ_PIN, pitch, 18);
    lastVarioMs = millis();
  }
}

// ================= ГРАФИЧЕСКИЕ ХЕЛПЕРЫ =================
inline uint16_t waterfallColor(uint8_t v) {
  if (v < 25)  return tft.color565(0, 0, 25 + v * 2);
  if (v < 75)  return tft.color565(0, (v - 25) * 3, 75 + (v - 25) * 3);
  if (v < 140) return tft.color565(0, 160 + (v - 75), 220 - (v - 75) * 3);
  if (v < 205) return tft.color565((v - 140) * 4, 255, 0);
  return tft.color565(255, 255 - (v - 205) * 4, (v - 205) * 4);
}

inline void drawHeader(const char* title, uint16_t bgCol, const char* rightBadge) {
  canvas.fillRect(0, 0, 320, 17, bgCol);
  canvas.drawFastHLine(0, 17, 320, TFT_WHITE);
  canvas.setTextColor(TFT_WHITE);
  canvas.setCursor(4, 4);
  canvas.print(title);
  int rLen = strlen(rightBadge) * 6;
  canvas.setCursor(316 - rLen, 4);
  canvas.setTextColor(TFT_YELLOW);
  canvas.print(rightBadge);
}

inline void drawFooter(int p1, int viewIdx, int maxViews, const char* hint) {
  canvas.fillRect(0, 155, 320, 15, 0x1082);
  canvas.drawFastHLine(0, 155, 320, 0x4208);
  canvas.setTextColor(TFT_CYAN);
  canvas.setCursor(4, 159);
  canvas.printf("POT:%02d%% V:%d/%d %s", p1, (viewIdx % maxViews) + 1, maxViews, soundEnabled ? "" : "[M]");
  canvas.setTextColor(TFT_LIGHTGREY);
  canvas.setCursor(108, 159);
  canvas.print(hint);
}

// ================= ПРЯМОЙ ДРАЙВЕР NRF24L01+ =================
inline uint8_t nrfSpi(uint8_t d) {
  for (int i = 0; i < 8; i++) {
    digitalWrite(NRF_MOSI, (d & 0x80) ? HIGH : LOW); d <<= 1;
    digitalWrite(NRF_SCK, HIGH); if (digitalRead(NRF_MISO)) d |= 1; digitalWrite(NRF_SCK, LOW);
  }
  return d;
}
inline void nrfWrite(uint8_t r, uint8_t v) {
  digitalWrite(NRF_CSN, LOW); nrfSpi(0x20 | (r & 0x1F)); nrfSpi(v); digitalWrite(NRF_CSN, HIGH);
}
inline uint8_t nrfRead(uint8_t r) {
  digitalWrite(NRF_CSN, LOW); nrfSpi(r & 0x1F); uint8_t v = nrfSpi(0xFF); digitalWrite(NRF_CSN, HIGH); return v;
}

inline void initNRF24() {
  pinMode(NRF_CE, OUTPUT); pinMode(NRF_CSN, OUTPUT); pinMode(NRF_SCK, OUTPUT);
  pinMode(NRF_MOSI, OUTPUT); pinMode(NRF_MISO, INPUT);
  digitalWrite(NRF_CE, LOW); digitalWrite(NRF_CSN, HIGH);
  nrfWrite(0x00, 0x03); delay(5); nrfWrite(0x01, 0x00); nrfWrite(0x06, 0x0F);
  nrfReg06 = nrfRead(0x06);
  hasNRF = (nrfReg06 == 0x0F || nrfReg06 == 0x0E);
}

// Быстрый замер активности одного канала NRF24 (0..125 -> 2400..2525 МГц)
inline bool sampleNrfChannel(uint8_t ch, uint16_t dwellUs = 200) {
  if (!hasNRF) return false;
  nrfWrite(0x05, ch);
  digitalWrite(NRF_CE, HIGH);
  delayMicroseconds(dwellUs);
  digitalWrite(NRF_CE, LOW);
  return (nrfRead(0x09) & 1) != 0;
}