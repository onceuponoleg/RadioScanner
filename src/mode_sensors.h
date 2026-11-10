#pragma once
#include "hardware.h"
#include "mode_rf.h"

// Данные Лазерной Лаборатории (Режим 3)
#define PROF_LEN 200
uint16_t profDistHist[PROF_LEN];
uint8_t  profGlintHist[PROF_LEN];
int profHead = 0;

#define LWF_H 38
uint8_t laserWfDist[LWF_H][100];
uint8_t laserWfGlint[LWF_H][100];
int lwfRow = 0;

float smoothLaserMM = 500.0f;
float prevLaserMM = 500.0f;
float laserVelocityCmS = 0.0f;
uint16_t baseTareMM = 500;
uint16_t rawLaserMM = 500;
uint8_t  rawLaserGlint = 15;
uint32_t lastLaserTimeMs = 0;

bool pointALocked = false;
float ptA_DistMM = 1000.0f;
float ptA_Yaw = 180.0f;
float ptA_Pitch = 0.0f;

// Данные Космоса GPS (Режим 5)
struct SatNode { uint8_t prn, elev; uint16_t azim; uint8_t snr; bool active; };
SatNode skySats[16];
String nmeaLine = "";
String nmeaHistory[6];
String lastPacketType = "NONE";

inline bool pollLaserLab() {
  uint16_t d = 0;
  uint8_t glint = 0;

  if (tofType == 1) {
    if ((i2cRead8(0x29, 0x13) & 0x07) == 0) return false;
    d = i2cRead16MSB(0x29, 0x1E);
    uint16_t rawMcps = i2cRead16MSB(0x29, 0x1A);
    i2cWrite(0x29, 0x0B, 0x01);
    if (d > 2200 || d < 20) d = 2200;
    float dNorm = constrain((int)d, 50, 2000) / 170.0f;
    glint = (d >= 2150) ? 5 : constrain((int)((rawMcps / 22.0f) * (dNorm * dNorm)), 5, 255);
  } else if (tofType == 2) {
    if (!tofL1.dataReady()) return false;
    tofL1.read(false);
    d = constrain((int)tofL1.ranging_data.range_mm, 20, 3000);
    float dNorm = constrain((int)d, 50, 2200) / 170.0f;
    glint = constrain((int)((tofL1.ranging_data.peak_signal_count_rate_MCPS * 5.0f) * (dNorm * dNorm)), 5, 255);
  } else {
    return false;
  }

  uint32_t now = millis();
  float dtSec = (now - lastLaserTimeMs) / 1000.0f;
  lastLaserTimeMs = now;

  rawLaserMM = d;
  rawLaserGlint = glint;
  prevLaserMM = smoothLaserMM;
  smoothLaserMM = smoothLaserMM * 0.65f + (float)d * 0.35f;

  if (dtSec > 0.005f && dtSec < 0.5f) {
    float rawVel = ((smoothLaserMM - prevLaserMM) * 0.1f) / dtSec;
    laserVelocityCmS = laserVelocityCmS * 0.7f + rawVel * 0.3f;
  }

  profDistHist[profHead]  = (uint16_t)smoothLaserMM;
  profGlintHist[profHead] = glint;
  profHead = (profHead + 1) % PROF_LEN;

  for (int x = 0; x < 100; x++) {
    int hIdx = (profHead - 100 + x + PROF_LEN) % PROF_LEN;
    laserWfDist[lwfRow][x]  = map(constrain((int)profDistHist[hIdx], 40, 1800), 1800, 40, 20, 250);
    laserWfGlint[lwfRow][x] = profGlintHist[hIdx];
  }
  lwfRow = (lwfRow + 1) % LWF_H;
  return true;
}

// ================= РЕЖИМ 0: ДИАГНОСТИКА =================
inline void runMode0(int gPct, int zPct, int view) {
  setSniffer(false);
  updateIMU();
  pollLaserLab();
  while (Serial2.available()) gps.encode(Serial2.read());

  canvas.fillSprite(TFT_BLACK);
  drawHeader("[0/7] SYSTEM DIAGNOSTICS", TFT_PURPLE, "HW CHECK");

  canvas.setCursor(6, 23);
  canvas.setTextColor(hasNRF ? TFT_GREEN : TFT_RED);
  canvas.printf("1. NRF24 SPI : %s (Reg06=0x%02X)", hasNRF ? "OK [ONLINE]" : "ERROR [CHECK WIRE]", nrfReg06);

  canvas.setCursor(6, 38);
  canvas.setTextColor(i2cCount > 0 ? TFT_CYAN : TFT_RED);
  canvas.print("2. I2C BUS   : ");
  if (i2cCount == 0) canvas.print("NONE FOUND!");
  for (int i = 0; i < i2cCount; i++) canvas.printf("0x%02X ", i2cList[i]);

  canvas.setCursor(6, 53);
  canvas.setTextColor(imuAddr ? TFT_GREEN : TFT_YELLOW);
  canvas.printf("3. BNO055 3D : Y:%3d P:%+3d R:%+3d deg", (int)panAngle, (int)tiltAngle, (int)rollAngle);

  canvas.setCursor(6, 68);
  canvas.setTextColor(tofType ? TFT_GREEN : TFT_RED);
  canvas.printf("4. LASER VL53: %4d mm | Glint:%3d", rawLaserMM, rawLaserGlint);

  canvas.setCursor(6, 83);
  canvas.setTextColor(gps.charsProcessed() > 20 ? TFT_GREEN : TFT_ORANGE);
  canvas.printf("5. GPS UART2 : %s (%u bytes RX)", gps.charsProcessed() > 20 ? "STREAMING" : "WAITING", gps.charsProcessed());

  canvas.setTextColor(TFT_WHITE);
  canvas.setCursor(6, 102);
  canvas.printf("POT1 (GAIN) : %3d%%", gPct);
  canvas.drawRect(120, 101, 192, 9, 0x4208);
  canvas.fillRect(122, 103, map(gPct, 0, 100, 0, 188), 5, TFT_CYAN);

  canvas.setCursor(6, 117);
  canvas.printf("POT2 (ZOOM) : %3d%%", zPct);
  canvas.drawRect(120, 116, 192, 9, 0x4208);
  canvas.fillRect(122, 118, map(zPct, 0, 100, 0, 188), 5, TFT_YELLOW);

  canvas.setCursor(6, 134);
  canvas.setTextColor(TFT_MAGENTA);
  canvas.printf("CONTROLS    : Btn=Mode | EncBtn=View | Sound=%s", soundEnabled ? "ON" : "MUTE");

  drawFooter(gPct, zPct, view, 1, "Btn:Next Mode | EncBtn:View");
  canvas.pushSprite(0, 0);
}

// ================= РЕЖИМ 3: ЛАЗЕРНАЯ ЛАБОРАТОРИЯ =================
inline void runMode3(int gPct, int zPct, int view) {
  setSniffer(false);
  updateIMU();
  pollLaserLab();

  int sub = view % 3;
  canvas.fillSprite(TFT_BLACK);

  if (sub == 0) {
    int glintThresh = map(gPct, 0, 100, 210, 30);
    int rangeSpanMM = map(zPct, 0, 100, 1500, 150);
    bool isGlintHit = (rawLaserGlint > glintThresh);
    if (isGlintHit) geigerTick(230);

    drawHeader("[3/7] LASER PROFILER & GLINT", isGlintHit ? TFT_MAGENTA : 0x0208,
               isGlintHit ? "OPTICS/METAL!" : "SURFACE SCOPE");

    int deltaMM = (int)smoothLaserMM - (int)baseTareMM;
    canvas.setCursor(4, 21);
    canvas.setTextColor(TFT_CYAN);
    canvas.printf("DIST:%4dmm (d:%+4dmm)", (int)smoothLaserMM, deltaMM);
    canvas.setCursor(164, 21);
    canvas.setTextColor(isGlintHit ? TFT_YELLOW : TFT_MAGENTA);
    canvas.printf("GLINT:%3d (Thr:%3d)", rawLaserGlint, glintThresh);

    canvas.drawRect(4, 32, 312, 74, 0x2104);
    canvas.drawFastHLine(6, 69, 308, 0x2104);

    int thrY = 104 - map(glintThresh, 0, 255, 2, 70);
    for (int x = 8; x < 310; x += 6) canvas.drawFastHLine(x, thrY, 3, 0x4008);

    for (int i = 0; i < PROF_LEN - 1; i++) {
      int idx1 = (profHead + i) % PROF_LEN;
      int idx2 = (profHead + i + 1) % PROF_LEN;
      int x1 = 6 + (i * 306) / PROF_LEN;
      int x2 = 6 + ((i + 1) * 306) / PROF_LEN;

      int d1 = constrain((int)profDistHist[idx1] - (int)baseTareMM, -rangeSpanMM, rangeSpanMM);
      int d2 = constrain((int)profDistHist[idx2] - (int)baseTareMM, -rangeSpanMM, rangeSpanMM);
      int yD1 = 69 - map(d1, -rangeSpanMM, rangeSpanMM, -33, 33);
      int yD2 = 69 - map(d2, -rangeSpanMM, rangeSpanMM, -33, 33);
      canvas.drawLine(x1, yD1, x2, yD2, TFT_CYAN);

      int g1 = profGlintHist[idx1];
      int g2 = profGlintHist[idx2];
      int yG1 = 104 - map(g1, 0, 255, 2, 70);
      int yG2 = 104 - map(g2, 0, 255, 2, 70);
      uint16_t gCol = (g2 > glintThresh) ? TFT_YELLOW : TFT_MAGENTA;
      canvas.drawLine(x1, yG1, x2, yG2, gCol);
    }

    canvas.drawRect(4, 110, 152, 42, 0x2104);
    canvas.drawRect(164, 110, 152, 42, 0x2104);
    for (int y = 0; y < LWF_H; y++) {
      int r = (lwfRow - 1 - y + LWF_H) % LWF_H;
      for (int x = 0; x < 74; x++) {
        int srcIdx = (x * 100) / 74;
        canvas.drawFastHLine(6 + x * 2, 112 + y, 2, waterfallColor(laserWfDist[r][srcIdx]));
        uint8_t gv = laserWfGlint[r][srcIdx];
        uint16_t gc = (gv > glintThresh) ? TFT_YELLOW : tft.color565(gv, 0, gv);
        canvas.drawFastHLine(166 + x * 2, 112 + y, 2, gc);
      }
    }
    drawFooter(gPct, zPct, sub, 3, "TurnEnc:Tare0 | EncBtn:Trig");

  } else if (sub == 1) {
    drawHeader("[3/7] 3D LASER TRIGONOMETRY", 0x0280, pointALocked ? "PT-A LOCKED" : "FREE MEASURE");

    int cx = 64, cy = 78, rad = 46;
    canvas.drawCircle(cx, cy, rad, 0x4208);
    canvas.drawCircle(cx, cy, 14, 0x2104);
    canvas.drawFastHLine(cx - rad, cy, rad * 2, 0x2104);
    canvas.drawFastVLine(cx, cy - rad, rad * 2, 0x2104);

    float rollRad = rollAngle * DEG_TO_RAD;
    int pitchOffset = constrain((int)(tiltAngle * 0.8f), -30, 30);
    int hx1 = cx - (int)(cosf(rollRad) * 38.0f);
    int hy1 = cy + pitchOffset - (int)(sinf(rollRad) * 38.0f);
    int hx2 = cx + (int)(cosf(rollRad) * 38.0f);
    int hy2 = cy + pitchOffset + (int)(sinf(rollRad) * 38.0f);
    bool isLevel = (fabs(tiltAngle) < 1.5f && fabs(rollAngle) < 1.5f);
    canvas.drawLine(hx1, constrain(hy1, 24, 130), hx2, constrain(hy2, 24, 130), isLevel ? TFT_GREEN : TFT_CYAN);

    int bubX = constrain(cx + (int)(rollAngle * 1.1f), cx - 36, cx + 36);
    int bubY = constrain(cy + (int)(tiltAngle * 1.1f), cy - 36, cy + 36);
    canvas.fillCircle(bubX, bubY, 4, isLevel ? TFT_GREEN : TFT_YELLOW);

    canvas.setCursor(6, 132);
    canvas.setTextColor(isLevel ? TFT_GREEN : TFT_WHITE);
    canvas.printf("PITCH:%+5.1f  ROLL:%+5.1f", tiltAngle, rollAngle);

    canvas.drawRect(126, 22, 190, 128, 0x4208);

    float d2 = smoothLaserMM;
    float vertCompMM  = d2 * sinf(tiltAngle * DEG_TO_RAD);
    float horizCompMM = d2 * cosf(tiltAngle * DEG_TO_RAD);

    if (!pointALocked) {
      canvas.setCursor(132, 27); canvas.setTextColor(TFT_YELLOW);
      canvas.printf("DIRECT RAY : %4d mm", (int)d2);
      canvas.setCursor(132, 42); canvas.setTextColor(TFT_CYAN);
      canvas.printf("HORIZ DIST : %4d mm", (int)horizCompMM);
      canvas.setCursor(132, 57); canvas.setTextColor(TFT_GREEN);
      canvas.printf("VERT HEIGHT: %+4d mm", (int)vertCompMM);

      canvas.drawLine(145, 125, 285, 125, TFT_CYAN);
      canvas.drawLine(285, 125, 285, 80, TFT_GREEN);
      canvas.drawLine(145, 125, 285, 80, TFT_YELLOW);
      canvas.setCursor(132, 134); canvas.setTextColor(TFT_MAGENTA);
      canvas.print("Turn Enc -> LOCK POINT [A]");
    } else {
      float d1 = ptA_DistMM;
      float dYaw = (panAngle - ptA_Yaw) * DEG_TO_RAD;
      float p1 = ptA_Pitch * DEG_TO_RAD;
      float p2 = tiltAngle * DEG_TO_RAD;

      float cosGamma = sinf(p1) * sinf(p2) + cosf(p1) * cosf(p2) * cosf(dYaw);
      cosGamma = constrain(cosGamma, -1.0f, 1.0f);
      float angleDeg = acosf(cosGamma) * RAD_TO_DEG;

      float spanAB_MM = sqrtf(max(0.0f, d1 * d1 + d2 * d2 - 2.0f * d1 * d2 * cosGamma));
      float deltaH_MM = d2 * sinf(p2) - d1 * sinf(p1);

      canvas.setCursor(132, 26); canvas.setTextColor(TFT_GREEN);
      canvas.printf("PT[A]:%4dmm | PT[B]:%4dmm", (int)d1, (int)d2);
      canvas.setCursor(132, 40); canvas.setTextColor(TFT_CYAN);
      canvas.printf("ANGLE A-O-B : %5.1f deg", angleDeg);

      canvas.fillRect(130, 54, 182, 26, 0x1082);
      canvas.setCursor(134, 58); canvas.setTextColor(TFT_YELLOW);
      canvas.printf("SPAN [A-B]: %4d mm (%.1fcm)", (int)spanAB_MM, spanAB_MM * 0.1f);
      canvas.setCursor(134, 70); canvas.setTextColor(TFT_WHITE);
      canvas.printf("HEIGHT dH : %+4d mm", (int)deltaH_MM);

      int ox = 220, oy = 138;
      int ax = ox - 45, ay = 96;
      int bx = constrain(ox + (int)((panAngle - ptA_Yaw) * 1.4f), 135, 305);
      int by = constrain(96 - (int)((tiltAngle - ptA_Pitch) * 0.8f), 84, 134);
      canvas.drawLine(ox, oy, ax, ay, TFT_GREEN);
      canvas.drawLine(ox, oy, bx, by, TFT_CYAN);
      canvas.drawLine(ax, ay, bx, by, TFT_YELLOW);
      canvas.fillCircle(ax, ay, 3, TFT_GREEN);
      canvas.fillCircle(bx, by, 3, TFT_YELLOW);
    }
    drawFooter(gPct, zPct, sub, 3, "TurnEnc:Lock PtA | EncBtn:Synth");

  } else {
    drawHeader("[3/7] DOPPLER & LASER THEREMIN", 0x300C, soundEnabled ? "SYNTH ON" : "SYNTH MUTE");

    int synthFreq = 0;
    if (rawLaserMM >= 45 && rawLaserMM <= 950) {
      int baseFreq = map(constrain((int)smoothLaserMM, 50, 900), 900, 50, 130, 1760);
      if (gPct > 50) {
        static const uint16_t scaleNotes[12] = {131, 147, 165, 196, 220, 262, 294, 330, 392, 440, 523, 659};
        int noteIdx = map(constrain((int)smoothLaserMM, 50, 850), 850, 50, 0, 11);
        int octave = (tiltAngle > 15.0f) ? 2 : 1;
        synthFreq = scaleNotes[noteIdx] * octave;
      } else {
        synthFreq = baseFreq + (int)(tiltAngle * 3.0f);
      }
      if (soundEnabled) tone(BUZZ_PIN, constrain(synthFreq, 110, 3500), 35);
    }

    int cx = 72, cy = 86;
    static float phaseShift = 0.0f;
    phaseShift = fmod(phaseShift - laserVelocityCmS * 0.12f + 100.0f, 18.0f);
    uint16_t ringCol = (laserVelocityCmS < -3.0f) ? TFT_RED : (laserVelocityCmS > 3.0f ? TFT_CYAN : TFT_GREEN);
    for (int r = (int)phaseShift; r < 56; r += 14) {
      if (r > 2) canvas.drawCircle(cx, cy, r, ringCol);
    }

    canvas.drawRect(140, 22, 174, 78, 0x2104);
    float waveFreq = 0.04f + (1000.0f / max((float)smoothLaserMM, 60.0f)) * 0.025f;
    float waveAmp  = map(constrain((int)rawLaserGlint, 10, 220), 10, 220, 8, 34);
    float phaseT   = millis() * 0.015f;

    for (int x = 0; x < 168; x++) {
      float y1 = sinf(x * waveFreq + phaseT) * cosf(x * 0.02f - tiltAngle * 0.05f) * waveAmp;
      float y2 = sinf((x + 1) * waveFreq + phaseT) * cosf((x + 1) * 0.02f - tiltAngle * 0.05f) * waveAmp;
      canvas.drawLine(143 + x, 61 + (int)y1, 144 + x, 61 + (int)y2, synthFreq > 0 ? TFT_MAGENTA : TFT_CYAN);
    }

    canvas.setCursor(140, 106); canvas.setTextColor(TFT_WHITE);
    canvas.printf("VELOCITY: %+6.1f cm/s", laserVelocityCmS);
    canvas.setCursor(140, 120); canvas.setTextColor(TFT_YELLOW);
    canvas.printf("DISTANCE: %4d mm", (int)smoothLaserMM);
    canvas.setCursor(140, 134); canvas.setTextColor(TFT_GREEN);
    canvas.printf("SYNTH HZ: %4d Hz (%s)", synthFreq, gPct > 50 ? "QUANTIZED" : "THEREMIN");

    drawFooter(gPct, zPct, sub, 3, "P1:Scale/Free | EncBtn:Prof");
  }

  canvas.pushSprite(0, 0);
}

// ================= РЕЖИМ 4: ВЕКТОРНЫЙ МАГНИТОСКОП =================
inline void runMode4(int gPct, int zPct, int view) {
  setSniffer(false);
  updateIMU();

  static int16_t histX[190], histY[190], histZ[190], histB[190];
  static int head = 0;

  float dx = magX - baseMagX;
  float dy = magY - baseMagY;
  float dz = magZ - baseMagZ;
  float totalB = sqrtf(dx * dx + dy * dy + dz * dz);

  float scale = 0.4f + (gPct / 12.0f);
  histX[head] = constrain((int)(dx * scale), -48, 48);
  histY[head] = constrain((int)(dy * scale), -48, 48);
  histZ[head] = constrain((int)(dz * scale), -48, 48);
  histB[head] = constrain((int)(totalB * scale), 0, 90);
  head = (head + 1) % 190;

  int alarmUt = map(zPct, 0, 100, 4, 140);
  if (totalB > alarmUt) {
    geigerTick(constrain((int)((totalB - alarmUt) * 5 + 50), 50, 255));
  }

  canvas.fillSprite(TFT_BLACK);
  const char* statusTxt = (totalB > alarmUt * 2) ? "STRONG FIELD!" : (totalB > alarmUt ? "ANOMALY" : "NORMAL");
  drawHeader("[4/7] EMF & MAGNETIC HUNTER", totalB > alarmUt ? TFT_RED : 0x3008, statusTxt);

  int cx = 56, cy = 82, r = 46;
  canvas.drawCircle(cx, cy, r, 0x4208);
  canvas.drawEllipse(cx, cy, r, r / 2, 0x2104);
  canvas.drawEllipse(cx, cy, r / 2, r, 0x2104);

  int vx = constrain((int)(dx * 1.5f), -r + 4, r - 4);
  int vy = constrain((int)(dy * 1.5f), -r + 4, r - 4);
  int vzRadius = constrain(3 + (int)(fabs(dz) * 0.2f), 3, 10);
  canvas.drawLine(cx, cy, cx + vx, cy + vy, TFT_YELLOW);
  canvas.fillCircle(cx + vx, cy + vy, vzRadius, totalB > alarmUt ? TFT_MAGENTA : TFT_CYAN);

  canvas.setCursor(8, 134);
  canvas.setTextColor(TFT_WHITE);
  canvas.printf("|B|:%.1f uT (Thr:%d)", totalB, alarmUt);

  canvas.drawRect(118, 22, 196, 126, 0x2104);
  canvas.drawFastHLine(120, 86, 192, 0x2104);

  canvas.setCursor(124, 26); canvas.setTextColor(TFT_RED);    canvas.printf("X:%.0f", dx);
  canvas.setCursor(172, 26); canvas.setTextColor(TFT_GREEN);  canvas.printf("Y:%.0f", dy);
  canvas.setCursor(220, 26); canvas.setTextColor(TFT_CYAN);   canvas.printf("Z:%.0f", dz);
  canvas.setCursor(268, 26); canvas.setTextColor(TFT_YELLOW); canvas.print("|B|");

  int thrY = 144 - constrain((int)(alarmUt * scale), 2, 105);
  for (int x = 122; x < 310; x += 6) canvas.drawFastHLine(x, thrY, 3, TFT_RED);

  for (int i = 0; i < 188; i++) {
    int i1 = (head + i) % 190;
    int i2 = (head + i + 1) % 190;
    int x = 122 + i;
    if (view % 2 == 0) {
      canvas.drawLine(x, 86 - histX[i1], x + 1, 86 - histX[i2], TFT_RED);
      canvas.drawLine(x, 86 - histY[i1], x + 1, 86 - histY[i2], TFT_GREEN);
      canvas.drawLine(x, 86 - histZ[i1], x + 1, 86 - histZ[i2], TFT_CYAN);
    } else {
      int by = 144 - histB[i1];
      canvas.drawFastVLine(x, by, 144 - by, (histB[i1] > alarmUt * scale) ? TFT_MAGENTA : 0x03E0);
    }
  }

  drawFooter(gPct, zPct, view, 2, "TurnEnc:Tare0 | EncBtn:View");
  canvas.pushSprite(0, 0);
}

// ================= РЕЖИМ 5: КОСМОС GPS =================
inline void parseNmeaLine(const String& s) {
  if (!s.startsWith("$")) return;
  for (int i = 5; i > 0; i--) nmeaHistory[i] = nmeaHistory[i - 1];
  nmeaHistory[0] = s.substring(0, 50);
  if (s.length() >= 6) lastPacketType = s.substring(1, 6);

  if (!s.startsWith("$GPGSV") && !s.startsWith("$GLGSV") && !s.startsWith("$GNGSV")) return;
  int cm[20], cnt = 0;
  for (unsigned int i = 0; i < s.length() && cnt < 20; i++) {
    if (s[i] == ',' || s[i] == '*') cm[cnt++] = i;
  }
  if (cnt < 7) return;
  int msgNum = s.substring(cm[1] + 1, cm[2]).toInt();
  int base = ((msgNum - 1) * 4) % 16;
  for (int k = 0; k < 4; k++) {
    int f = 3 + k * 4;
    if (f + 3 >= cnt) break;
    int prn  = s.substring(cm[f] + 1, cm[f + 1]).toInt();
    int elev = s.substring(cm[f + 1] + 1, cm[f + 2]).toInt();
    int azim = s.substring(cm[f + 2] + 1, cm[f + 3]).toInt();
    int snr  = (f + 4 < cnt) ? s.substring(cm[f + 3] + 1, cm[f + 4]).toInt() : 0;
    if (prn > 0 && base + k < 16) {
      skySats[base + k] = {(uint8_t)prn, (uint8_t)elev, (uint16_t)azim, (uint8_t)snr, true};
    }
  }
}

inline void runMode5(int gPct, int zPct, int view) {
  setSniffer(false);
  while (Serial2.available()) {
    char c = Serial2.read();
    gps.encode(c);
    if (c == '\n') {
      parseNmeaLine(nmeaLine);
      nmeaLine = "";
    } else if (c != '\r' && nmeaLine.length() < 100) {
      nmeaLine += c;
    }
  }

  int activeSatList[16];
  int satCnt = 0;
  for (int i = 0; i < 16; i++) if (skySats[i].active) activeSatList[satCnt++] = i;

  canvas.fillSprite(TFT_BLACK);
  char badge[20];
  snprintf(badge, sizeof(badge), "SATS:%d", satCnt);
  drawHeader("[5/7] ORBITAL SPACE TRACKER", TFT_DARKGREEN, badge);

  if (view % 2 == 0) {
    float zoomFactor = 1.0f + (zPct / 35.0f);
    int selectedSat = (satCnt > 0) ? map(gPct, 0, 100, 0, satCnt - 1) : 0;

    int cx = 72, cy = 86, maxR = 56;
    canvas.drawCircle(cx, cy, maxR, TFT_GREEN);
    canvas.drawCircle(cx, cy, maxR / 2, 0x2104);
    canvas.drawLine(cx - maxR, cy, cx + maxR, cy, 0x2104);
    canvas.drawLine(cx, cy - maxR, cx, cy + maxR, 0x2104);
    canvas.setTextColor(TFT_GREEN);
    canvas.setCursor(cx - 18, cy - maxR + 3);
    canvas.printf("%.1fx", zoomFactor);

    for (int k = 0; k < satCnt; k++) {
      int idx = activeSatList[k];
      float rawR = map(constrain((int)skySats[idx].elev, 0, 90), 0, 90, maxR - 2, 2);
      float zoomedR = rawR / zoomFactor;
      float a = (skySats[idx].azim - 90.0f) * DEG_TO_RAD;
      int px = cx + cos(a) * zoomedR;
      int py = cy + sin(a) * zoomedR;

      if (hypot(px - cx, py - cy) < maxR) {
        uint16_t col = (skySats[idx].snr > 0) ? TFT_YELLOW : TFT_CYAN;
        canvas.fillCircle(px, py, 3, col);
        if (k == selectedSat) {
          canvas.drawCircle(px, py, 7, TFT_MAGENTA);
          canvas.drawLine(cx, cy, px, py, 0x4208);
        }
      }
    }

    canvas.drawRect(138, 22, 178, 56, 0x4208);
    if (satCnt > 0) {
      SatNode& s = skySats[activeSatList[selectedSat]];
      const char* sysName = (s.prn >= 65) ? "RUS GLONASS" : (s.prn > 32 ? "SBAS/GALILEO" : "USA NAVSTAR");
      canvas.setCursor(144, 26); canvas.setTextColor(TFT_YELLOW);
      canvas.printf("SAT PRN #%02d [%s]", s.prn, sysName);
      canvas.setCursor(144, 40); canvas.setTextColor(TFT_WHITE);
      canvas.printf("AZIMUTH:%3d deg | ELEV:%2d deg", s.azim, s.elev);
      canvas.setCursor(144, 54); canvas.setTextColor(s.snr > 0 ? TFT_GREEN : TFT_CYAN);
      canvas.printf("SIGNAL SNR: %2d dBHz", s.snr);
      canvas.fillRect(262, 54, map(constrain((int)s.snr, 0, 50), 0, 50, 2, 48), 7, TFT_GREEN);
    } else {
      canvas.setCursor(144, 42); canvas.setTextColor(TFT_ORANGE);
      canvas.print("WAITING FOR ALMANAC...");
    }

    canvas.setTextColor(TFT_WHITE);
    canvas.setCursor(138, 86);  canvas.printf("UTC TIME : %02d:%02d:%02d", gps.time.hour(), gps.time.minute(), gps.time.second());
    canvas.setCursor(138, 102); canvas.printf("LAT/LON  : %.4f / %.4f", gps.location.lat(), gps.location.lng());
    canvas.setCursor(138, 118); canvas.printf("RX STREAM: %u bytes (%s)", gps.charsProcessed(), lastPacketType.c_str());
    canvas.setCursor(138, 134); canvas.setTextColor(gps.location.isValid() ? TFT_GREEN : TFT_YELLOW);
    canvas.printf("STATUS   : %s", gps.location.isValid() ? "3D ORBIT LOCK" : "INDOOR ALMANAC RX");

    drawFooter(gPct, zPct, view, 2, "EncBtn:Terminal | P2:Zoom");
  } else {
    canvas.setTextColor(TFT_YELLOW);
    canvas.setCursor(4, 22);
    canvas.printf("LIVE NMEA STREAM (RX:%u B) | LAST:%s", gps.charsProcessed(), lastPacketType.c_str());
    canvas.drawFastHLine(0, 32, 320, 0x4208);

    for (int i = 0; i < 6; i++) {
      canvas.setCursor(4, 37 + i * 16);
      canvas.setTextColor(i == 0 ? TFT_GREEN : 0x03E0);
      canvas.print(nmeaHistory[i].length() > 0 ? nmeaHistory[i] : "Waiting UART2 data...");
    }

    canvas.fillRect(0, 134, 320, 19, 0x1082);
    canvas.setCursor(4, 139);
    canvas.setTextColor(TFT_CYAN);
    canvas.printf("DECODED -> UTC:%02d:%02d:%02d SatsInView:%d CharsOK:%u",
                  gps.time.hour(), gps.time.minute(), gps.time.second(), satCnt, gps.charsProcessed());

    drawFooter(gPct, zPct, view, 2, "EncBtn:Back to Sky Dome");
  }
  canvas.pushSprite(0, 0);
}