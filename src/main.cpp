#include <Arduino.h>
#include "config.h"
#include "hardware.h"
#include "sniffer_engine.h"
#include "modes_rf.h"
#include "mode_net.h"

void setup() {
  Serial.begin(115200);

  pinMode(BUZZ_PIN, OUTPUT);
  pinMode(ENC_S1, INPUT_PULLUP);
  pinMode(ENC_S2, INPUT_PULLUP);
  pinMode(ENC_KEY, INPUT_PULLUP);
  pinMode(BTN_MODE, INPUT_PULLUP);

  encState = ((digitalRead(ENC_S1) << 1) | digitalRead(ENC_S2)) & 0x03;
  attachInterrupt(digitalPinToInterrupt(ENC_S1), encQuadISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_S2), encQuadISR, CHANGE);

  tft.init();
  tft.setRotation(1);
  canvas.setColorDepth(8);
  canvas.createSprite(320, 170);

  initNRF24();

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
}

void loop() {
  // 1. КНОПКА РЕЖИМОВ (BTN_MODE, GPIO 5): переключает 6 главных режимов
  static uint32_t modePressMs = 0;
  static bool modeWasDown = false;
  bool modeDown = (digitalRead(BTN_MODE) == LOW);

  if (modeDown && !modeWasDown) {
    modeWasDown = true;
    modePressMs = millis();
  } else if (!modeDown && modeWasDown) {
    uint32_t dur = millis() - modePressMs;
    modeWasDown = false;
    if (dur > 700) {
      soundEnabled = !soundEnabled;
      tone(BUZZ_PIN, 900, 90);
    } else if (dur > 25) {
      currentMode = (currentMode + 1) % TOTAL_MODES;
      subView = 0;
      playClick(2400, 18);
    }
  }

  // 2. КНОПКА ЭНКОДЕРА (ENC_KEY, GPIO 19): переключает вьюшки V:1, V:2, V:3 внутри режима
  static uint32_t encPressMs = 0;
  static bool encWasDown = false;
  bool encDown = (digitalRead(ENC_KEY) == LOW);

  if (encDown && !encWasDown) {
    encWasDown = true;
    encPressMs = millis();
  } else if (!encDown && encWasDown) {
    uint32_t dur = millis() - encPressMs;
    encWasDown = false;
    if (dur > 600) {
      memset(peakHold, 0, sizeof(peakHold));
      memset(probeLog, 0, sizeof(probeLog));
      evilTwinDetected = false;
      cntDeauth = 0;
      resetNetStats();
      playClick(1000, 70);
    } else if (dur > 25) {
      subView = (subView + 1) % 6;
      playClick(1700, 15);
    }
  }

  // 3. Вращение вала энкодера (скролл таблиц, выбор цели для пеленгации, смена сервера)
  static int32_t lastDetents = 1000;
  int32_t curDetents = encDetents;
  if (curDetents != lastDetents) {
    lastDetents = curDetents;
    if (currentMode == 5) {
      wanTargetIdx = (wanTargetIdx + 1) % 3;
    }
    playClick(1300, 8);
  }

  // 4. Чтение потенциометра с защитой от дребезга АЦП
  static int lockedAdc1 = -100, lockedAdc2 = -100;
  static int gPct = 50, zPct = 50;

  int raw1 = (analogRead(POT_GAIN) + analogRead(POT_GAIN)) / 2;
  int raw2 = (analogRead(POT_ZOOM) + analogRead(POT_ZOOM)) / 2;

  if (abs(raw1 - lockedAdc1) > 48 || raw1 < 25 || raw1 > 4070) {
    lockedAdc1 = raw1;
    gPct = constrain(map(raw1, 30, 4060, 0, 100), 0, 100);
  }
  if (abs(raw2 - lockedAdc2) > 48 || raw2 < 25 || raw2 > 4070) {
    lockedAdc2 = raw2;
    zPct = constrain(map(raw2, 30, 4060, 0, 100), 0, 100);
  }

  switch (currentMode) {
    case 0: runMode1_Spectrum(gPct, zPct, subView);               break;
    case 1: runMode2_Sniffer(gPct, zPct, subView, curDetents);    break;
    case 2: runMode3_FoxHunt(gPct, zPct, subView, curDetents);    break;
    case 3: runMode4_Security(gPct, zPct, subView, curDetents);   break;
    case 4: runMode5_ProtocolLab(gPct, zPct, subView);            break;
    case 5: runMode6_NetQuality(gPct, zPct, subView);             break;
  }
}