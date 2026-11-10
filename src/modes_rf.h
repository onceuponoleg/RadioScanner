#pragma once
#include "hardware.h"
#include "sniffer_engine.h"

#define NRF_CHANNELS 126
uint8_t spectrum[NRF_CHANNELS];
uint8_t peakHold[NRF_CHANNELS];
#define WF_HEIGHT 78
uint8_t waterfallBuf[WF_HEIGHT][NRF_CHANNELS];
int wfRow = 0;
uint8_t chPersistence[14] = {0};

// Данные для Режима 3 (Пеленгатор Fox-Hunt)
#define FOX_HIST 200
int8_t foxRssiHist[FOX_HIST];
int    foxHead = 0;
int    foxTargetNodeIdx = 0;

// Данные для Режима 5 (Лаборатория не-Wi-Fi протоколов 2.4 ГГц)
uint8_t bleAdvHits[3] = {0};         // Ch 37 (2402MHz), Ch 38 (2426MHz), Ch 39 (2480MHz)
uint8_t upperIsmSpec[46] = {0};      // 2480..2525 MHz (вне диапазона Wi-Fi!)
uint8_t upperIsmWf[70][46] = {0};
int     upperWfRow = 0;

// ================= РЕЖИМ 1: 2.4GHz SDR SPECTRUM & WATERFALL =================
inline void runMode1_Spectrum(int gPct, int zPct, int view) {
  setSniffer(true);
  tickSnifferStats1Hz();

  static int hopCh = 1;
  esp_wifi_set_channel(hopCh, WIFI_SECOND_CHAN_NONE);
  hopCh = (hopCh % 13) + 1;

  float gain = 0.7f + (gPct / 18.0f);
  int squelch = map(zPct, 0, 100, 0, 45);
  int maxSig = 0, peakMhz = 2400;

  uint8_t rawSpec[NRF_CHANNELS];
  for (int ch = 0; ch < NRF_CHANNELS; ch++) {
    int val = random(8, 20);
    if (sampleNrfChannel(ch, 190)) val += 165;

    for (int w = 1; w <= 13; w++) {
      int center = 12 + (w - 1) * 5;
      int d = abs(ch - center);
      if (d <= 10 && chPktHits[w] > 0) {
        int rssiBoost = constrain(map(chPeakRssi[w], -95, -35, 45, 240), 30, 240);
        int dome = (rssiBoost * (11 - d)) / 11;
        if (dome > val) val = dome;
      }
    }
    rawSpec[ch] = constrain((int)(val * gain) - squelch, 8, 255);
  }

  for (int ch = 0; ch < NRF_CHANNELS; ch++) {
    int prev = (ch > 0) ? rawSpec[ch - 1] : rawSpec[ch];
    int next = (ch < NRF_CHANNELS - 1) ? rawSpec[ch + 1] : rawSpec[ch];
    spectrum[ch] = (spectrum[ch] + (prev + rawSpec[ch] * 2 + next) / 4) / 2;

    if (spectrum[ch] > peakHold[ch]) peakHold[ch] = spectrum[ch];
    else if (peakHold[ch] > 1) peakHold[ch] -= 1;

    if (spectrum[ch] > maxSig) { maxSig = spectrum[ch]; peakMhz = 2400 + ch; }
    waterfallBuf[wfRow][ch] = spectrum[ch];
  }

  wfRow = (wfRow + 1) % WF_HEIGHT;
  geigerTick(maxSig > 45 ? maxSig : 0);

  canvas.fillSprite(TFT_BLACK);
  char badge[20];
  snprintf(badge, sizeof(badge), "PK:%dMHz", peakMhz);
  drawHeader("[1/6] 2.4GHz SDR SPECTRUM", 0x0186, badge);

  if (view % 2 == 0) {
    for (int ch = 0; ch < NRF_CHANNELS - 1; ch++) {
      int x1 = 4 + (ch * 312) / NRF_CHANNELS;
      int x2 = 4 + ((ch + 1) * 312) / NRF_CHANNELS;
      int y1 = 56 - map(spectrum[ch], 0, 255, 0, 36);
      int y2 = 56 - map(spectrum[ch + 1], 0, 255, 0, 36);
      int py = 56 - map(peakHold[ch], 0, 255, 0, 36);
      canvas.drawFastVLine(x1, y1, 56 - y1, 0x0210);
      canvas.drawLine(x1, y1, x2, y2, TFT_CYAN);
      canvas.drawPixel(x1, py, TFT_YELLOW);
    }

    canvas.fillRect(0, 58, 320, 14, 0x1082);
    for (int w = 1; w <= 13; w++) {
      int cx = 4 + ((12 + (w - 1) * 5) * 312) / NRF_CHANNELS;
      if (chPktHits[w] > 0) {
        uint8_t t = chLastType[w];
        uint16_t pCol = (t == 2) ? TFT_MAGENTA : (t == 3 ? TFT_YELLOW : TFT_CYAN);
        char pChar = (t == 2) ? 'P' : (t == 3 ? 'D' : 'B');
        canvas.fillRect(cx - 5, 59, 11, 12, pCol);
        canvas.setTextColor(TFT_BLACK);
        canvas.setCursor(cx - 2, 61);
        canvas.print(pChar);
        if (chPersistence[w] < 240) chPersistence[w] = min(255, chPersistence[w] + 45);
        chPktHits[w] /= 2;
      } else {
        chPeakRssi[w] = -100;
        if (chPersistence[w] > 2) chPersistence[w] -= 2;
        canvas.setTextColor(0x632C);
        canvas.setCursor(cx - 3, 61);
        canvas.printf("%d", w);
      }
    }

    for (int y = 0; y < WF_HEIGHT; y++) {
      int r = (wfRow - 1 - y + WF_HEIGHT) % WF_HEIGHT;
      for (int ch = 0; ch < NRF_CHANNELS; ch++) {
        int x = 4 + (ch * 312) / NRF_CHANNELS;
        canvas.drawFastHLine(x, 74 + y, 3, waterfallColor(waterfallBuf[r][ch]));
      }
    }
    drawFooter(gPct, view, 2, "Pot:Gain | EncBtn:13Ch View");
  } else {
    int bestCh = 1, worstCh = 1, minLoad = 9999, maxLoad = -1;
    for (int w = 1; w <= 13; w++) {
      int center = 12 + (w - 1) * 5;
      int load = (spectrum[center] + chPersistence[w]) / 2;
      if (load < minLoad) { minLoad = load; bestCh = w; }
      if (load > maxLoad) { maxLoad = load; worstCh = w; }

      int bx = 10 + (w - 1) * 23;
      int barH = map(constrain(load, 0, 255), 0, 255, 4, 88);
      int pkH  = map(constrain((int)peakHold[center], 0, 255), 0, 255, 4, 88);

      canvas.drawRect(bx, 42, 18, 90, 0x2104);
      for (int sy = 0; sy < barH; sy += 3) {
        canvas.fillRect(bx + 2, 130 - sy, 14, 2, waterfallColor(map(sy, 0, 88, 40, 250)));
      }
      canvas.drawFastHLine(bx + 1, 130 - pkH, 16, TFT_WHITE);
      canvas.setTextColor(w == worstCh ? TFT_RED : (w == bestCh ? TFT_GREEN : TFT_WHITE));
      canvas.setCursor(bx + (w < 10 ? 6 : 3), 136);
      canvas.printf("%d", w);
    }
    canvas.setCursor(6, 23);  canvas.setTextColor(TFT_GREEN);  canvas.printf("CLEANEST CH: #%02d", bestCh);
    canvas.setCursor(135, 23);canvas.setTextColor(TFT_ORANGE); canvas.printf("BUSIEST: #%02d | Pkts:%u", worstCh, cntBeacons + cntProbes + cntData);
    drawFooter(gPct, view, 2, "EncBtn:Back to Waterfall");
  }
  canvas.pushSprite(0, 0);
}

// ================= РЕЖИМ 2: 802.11 SNIFFER, RADAR & PROBE CATCHER =================
inline void runMode2_Sniffer(int gPct, int zPct, int view, int32_t encSteps) {
  setSniffer(true);
  tickSnifferStats1Hz();

  static uint32_t lastHop = 0;
  static int scanCh = 1;
  if (millis() - lastHop > 170) {
    lastHop = millis();
    scanCh = (scanCh % 13) + 1;
    esp_wifi_set_channel(scanCh, WIFI_SECOND_CHAN_NONE);
  }

  int activeIdx[MAX_NODES];
  int activeCnt = 0;
  for (int i = 0; i < MAX_NODES; i++) {
    if (nodeTable[i].pktCount > 0 && (millis() - nodeTable[i].lastSeen < 30000)) {
      activeIdx[activeCnt++] = i;
    }
  }

  canvas.fillSprite(TFT_BLACK);
  int sub = view % 3;

  if (sub == 0) {
    // V:1/3 — Радар устройств + Досье цели (Энкодер переключает цели, Потенциометр зумирует радар!)
    char badge[20];
    snprintf(badge, sizeof(badge), "NODES:%d", activeCnt);
    drawHeader("[2/6] 802.11 DEVICE SONAR", TFT_DARKCYAN, badge);

    float maxRadarMeters = map(gPct, 0, 100, 35, 3);
    int selectedTarget = (activeCnt > 0) ? (abs((int)encSteps) % activeCnt) : 0;
    foxTargetNodeIdx = (activeCnt > 0) ? activeIdx[selectedTarget] : 0;

    int cx = 74, cy = 86, rad = 58;
    canvas.drawCircle(cx, cy, rad, 0x03E0);
    canvas.drawCircle(cx, cy, rad / 2, 0x2104);
    canvas.drawLine(cx - rad, cy, cx + rad, cy, 0x2104);
    canvas.drawLine(cx, cy - rad, cx, cy + rad, 0x2104);

    float sweepAng = (millis() % 2800) * 0.128f * DEG_TO_RAD;
    canvas.drawLine(cx, cy, cx + cos(sweepAng) * rad, cy + sin(sweepAng) * rad, TFT_GREEN);
    canvas.setTextColor(TFT_GREEN);
    canvas.setCursor(cx - 12, cy - rad + 3);
    canvas.printf("%.0fm", maxRadarMeters);

    for (int k = 0; k < activeCnt; k++) {
      int idx = activeIdx[k];
      float distM = powf(10.0f, (-41.0f - nodeTable[idx].smoothRssi) / 22.0f);
      float normR = constrain(distM / maxRadarMeters, 0.08f, 0.95f) * rad;
      float ang = (nodeTable[idx].mac[5] * 23 + nodeTable[idx].channel * 27) * DEG_TO_RAD;
      int px = cx + cos(ang) * normR;
      int py = cy + sin(ang) * normR;

      uint8_t ft = nodeTable[idx].frameType;
      uint16_t col = (ft == 1) ? TFT_MAGENTA : (ft == 2 ? TFT_YELLOW : TFT_CYAN);
      canvas.fillCircle(px, py, 3, col);

      if (k == selectedTarget) {
        canvas.drawCircle(px, py, 7, TFT_WHITE);
        canvas.drawLine(cx, cy, px, py, 0x4208);
      }
    }

    if (activeCnt > 0) {
      int sel = activeIdx[selectedTarget];
      float dM = powf(10.0f, (-41.0f - nodeTable[sel].smoothRssi) / 22.0f);
      canvas.drawRect(140, 22, 176, 54, TFT_WHITE);
      canvas.setCursor(145, 26); canvas.setTextColor(TFT_YELLOW);
      canvas.printf("TARGET [%d/%d] CH:%02d", selectedTarget + 1, activeCnt, nodeTable[sel].channel);
      canvas.setCursor(145, 38); canvas.setTextColor(TFT_WHITE);
      canvas.printf("MAC: %02X:%02X:%02X:%02X:%02X:%02X",
                    nodeTable[sel].mac[0], nodeTable[sel].mac[1], nodeTable[sel].mac[2],
                    nodeTable[sel].mac[3], nodeTable[sel].mac[4], nodeTable[sel].mac[5]);
      canvas.setCursor(145, 50); canvas.setTextColor(TFT_CYAN);
      canvas.printf("TYPE: %s (%upps)", getVendorName(nodeTable[sel].mac, nodeTable[sel].frameType), nodeTable[sel].dataPps);
      canvas.setCursor(145, 62); canvas.setTextColor(TFT_GREEN);
      canvas.printf("RSSI:%ddB (~%.1fm) %s", (int)nodeTable[sel].smoothRssi, dM, nodeTable[sel].ssid);
    }

    for (int r = 0; r < 4; r++) {
      int k = (selectedTarget + r) % max(activeCnt, 1);
      if (k >= activeCnt) break;
      int idx = activeIdx[k];
      canvas.setCursor(140, 82 + r * 17);
      canvas.setTextColor(k == selectedTarget ? TFT_YELLOW : TFT_LIGHTGREY);
      canvas.printf("%c%02X:%02X %3ddB %s", k == selectedTarget ? '>' : ' ',
                    nodeTable[idx].mac[4], nodeTable[idx].mac[5], (int)nodeTable[idx].smoothRssi,
                    nodeTable[idx].ssid[0] ? nodeTable[idx].ssid : getVendorName(nodeTable[idx].mac, nodeTable[idx].frameType));
    }
    drawFooter(gPct, sub, 3, "Enc:Select Target | Pot:Zoom");

  } else if (sub == 1) {
    // V:2/3 — Полная таблица MAC/SSID со скроллом на Энкодере
    drawHeader("[2/6] 802.11 LIVE MAC TABLE", TFT_DARKCYAN, "SCROLLABLE");
    canvas.setTextColor(TFT_YELLOW);
    canvas.setCursor(4, 22);
    canvas.print("MAC / SSID       CH RSSI  PPS VENDOR/TYPE");
    canvas.drawFastHLine(0, 32, 320, 0x4208);

    int maxScroll = max(0, activeCnt - 6);
    int scrollOffset = (maxScroll > 0) ? (abs((int)encSteps) % (maxScroll + 1)) : 0;

    for (int r = 0; r < 6; r++) {
      int k = scrollOffset + r;
      if (k >= activeCnt) break;
      int idx = activeIdx[k];
      int y = 36 + r * 19;

      uint8_t ft = nodeTable[idx].frameType;
      canvas.setTextColor(ft == 1 ? TFT_MAGENTA : (nodeTable[idx].dataPps > 5 ? TFT_YELLOW : TFT_GREEN));
      canvas.setCursor(4, y);
      if (nodeTable[idx].ssid[0] != '\0') {
        canvas.printf("%-16s %02d %3ddB %3u %s",
                      nodeTable[idx].ssid, nodeTable[idx].channel, (int)nodeTable[idx].smoothRssi,
                      nodeTable[idx].dataPps, getVendorName(nodeTable[idx].mac, ft));
      } else {
        canvas.printf("%02X:%02X:%02X:%02X:%02X:%02X %02d %3ddB %3u %s",
                      nodeTable[idx].mac[0], nodeTable[idx].mac[1], nodeTable[idx].mac[2],
                      nodeTable[idx].mac[3], nodeTable[idx].mac[4], nodeTable[idx].mac[5],
                      nodeTable[idx].channel, (int)nodeTable[idx].smoothRssi,
                      nodeTable[idx].dataPps, getVendorName(nodeTable[idx].mac, ft));
      }
    }
    drawFooter(gPct, sub, 3, "Turn Enc:Scroll | EncBtn:Probes");

  } else {
    // V:3/3 — PROBE REQUEST CATCHER (Какие домашние сети ищут телефоны людей вокруг)
    int probeCnt = 0;
    for (int i = 0; i < MAX_PROBES; i++) if (probeLog[i].count > 0) probeCnt++;
    char badge[20];
    snprintf(badge, sizeof(badge), "SSIDS:%d", probeCnt);
    drawHeader("[2/6] PHONE PROBE CATCHER", 0x4008, badge);

    canvas.setTextColor(TFT_YELLOW);
    canvas.setCursor(4, 22);
    canvas.print("PHONE MAC         RSSI  HITS  SEARCHED WIFI SSID");
    canvas.drawFastHLine(0, 32, 320, 0x4208);

    int row = 0;
    for (int i = 0; i < MAX_PROBES && row < 6; i++) {
      if (probeLog[i].count == 0) continue;
      canvas.setCursor(4, 37 + row * 19);
      canvas.setTextColor(TFT_MAGENTA);
      canvas.printf("%02X:%02X:%02X:%02X:%02X:%02X ",
                    probeLog[i].phoneMac[0], probeLog[i].phoneMac[1], probeLog[i].phoneMac[2],
                    probeLog[i].phoneMac[3], probeLog[i].phoneMac[4], probeLog[i].phoneMac[5]);
      canvas.setTextColor(TFT_CYAN);
      canvas.printf("%4ddB %4u  ", probeLog[i].rssi, probeLog[i].count);
      canvas.setTextColor(TFT_WHITE);
      canvas.print(probeLog[i].probedSsid);
      row++;
    }
    if (row == 0) {
      canvas.setCursor(20, 80);
      canvas.setTextColor(TFT_LIGHTGREY);
      canvas.print("Listening for Smartphone Probe Requests...");
    }
    drawFooter(gPct, sub, 3, "Shows Wi-Fi names phones seek!");
  }
  canvas.pushSprite(0, 0);
  (void)zPct;
}

// ================= РЕЖИМ 3: FOX-HUNT PINPOINTER (НАПРАВЛЕННЫЙ ПЕЛЕНГАТОР ИСТОЧНИКА) =================
inline void runMode3_FoxHunt(int gPct, int zPct, int view, int32_t encSteps) {
  setSniffer(true);
  tickSnifferStats1Hz();

  int activeIdx[MAX_NODES];
  int activeCnt = 0;
  for (int i = 0; i < MAX_NODES; i++) {
    if (nodeTable[i].pktCount > 0 && (millis() - nodeTable[i].lastSeen < 40000)) {
      activeIdx[activeCnt++] = i;
    }
  }

  int sub = view % 2;
  canvas.fillSprite(TFT_BLACK);

  if (sub == 0) {
    // V:1/2 — Пеленгация конкретного Wi-Fi передатчика (Роутера / Скрытой камеры / Телефона)
    int sel = (activeCnt > 0) ? activeIdx[abs((int)encSteps) % activeCnt] : 0;
    uint8_t lockedCh = (activeCnt > 0) ? constrain((int)nodeTable[sel].channel, 1, 13) : 6;
    // Фиксируем радиоканал без прыжков для 10-кратного ускорения замеров RSSI!
    esp_wifi_set_channel(lockedCh, WIFI_SECOND_CHAN_NONE);

    int curRssi = (activeCnt > 0) ? (int)nodeTable[sel].smoothRssi : -95;
    int pkRssi  = (activeCnt > 0) ? (int)nodeTable[sel].peakRssi   : -95;
    int threshDbm = map(gPct, 0, 100, -92, -35);

    foxRssiHist[foxHead] = constrain(curRssi, -98, -20);
    foxHead = (foxHead + 1) % FOX_HIST;

    variometerBeep(curRssi, threshDbm);

    char badge[20];
    snprintf(badge, sizeof(badge), "LOCK CH:%02d", lockedCh);
    drawHeader("[3/6] FOX-HUNT PINPOINTER", curRssi > threshDbm ? TFT_RED : 0x0280, badge);

    // Карточка пеленгуемой цели
    canvas.setCursor(6, 22);
    canvas.setTextColor(TFT_YELLOW);
    if (activeCnt > 0) {
      canvas.printf("TARGET: %02X:%02X:%02X:%02X:%02X:%02X (%s)",
                    nodeTable[sel].mac[0], nodeTable[sel].mac[1], nodeTable[sel].mac[2],
                    nodeTable[sel].mac[3], nodeTable[sel].mac[4], nodeTable[sel].mac[5],
                    nodeTable[sel].ssid[0] ? nodeTable[sel].ssid : getVendorName(nodeTable[sel].mac, nodeTable[sel].frameType));
    } else {
      canvas.print("NO TARGETS YET (WAIT FOR SNIFFER)");
    }

    // Гигантский прогресс-бар близости сигнала (-95 .. -25 dBm)
    int barFill = map(constrain(curRssi, -95, -25), -95, -25, 4, 210);
    int pkPos   = map(constrain(pkRssi,  -95, -25), -95, -25, 4, 210);
    canvas.drawRect(6, 36, 214, 22, TFT_WHITE);
    for (int x = 0; x < barFill; x += 3) {
      canvas.fillRect(8 + x, 38, 2, 18, waterfallColor(map(x, 0, 210, 35, 250)));
    }
    canvas.drawFastVLine(8 + pkPos, 37, 20, TFT_YELLOW);

    float estMeters = powf(10.0f, (-40.0f - curRssi) / 22.0f);
    canvas.setCursor(226, 36); canvas.setTextColor(TFT_WHITE);  canvas.printf("NOW :%3ddBm", curRssi);
    canvas.setCursor(226, 48); canvas.setTextColor(TFT_YELLOW); canvas.printf("DIST:~%.1fm", estMeters);

    // Быстрая лента-самописец уровня сигнала (позволяет видеть малейший шаг к источнику!)
    canvas.drawRect(6, 64, 308, 86, 0x2104);
    int thrY = 148 - map(constrain(threshDbm, -95, -25), -95, -25, 4, 80);
    for (int x = 10; x < 310; x += 6) canvas.drawFastHLine(x, thrY, 3, TFT_RED);

    for (int i = 0; i < FOX_HIST - 1; i++) {
      int i1 = (foxHead + i) % FOX_HIST;
      int i2 = (foxHead + i + 1) % FOX_HIST;
      int x1 = 8 + (i * 302) / FOX_HIST;
      int x2 = 8 + ((i + 1) * 302) / FOX_HIST;
      int y1 = 148 - map(constrain((int)foxRssiHist[i1], -95, -25), -95, -25, 4, 80);
      int y2 = 148 - map(constrain((int)foxRssiHist[i2], -95, -25), -95, -25, 4, 80);
      canvas.drawLine(x1, y1, x2, y2, foxRssiHist[i2] > threshDbm ? TFT_YELLOW : TFT_CYAN);
    }
    drawFooter(gPct, sub, 2, "Enc:Change Target | Pot:AudioThr");

  } else {
    // V:2/2 — Широкополосный физический РФ-детектор поля NRF24 (ищет даже аналоговые камеры и жучки без Wi-Fi!)
    int centerMhz = 2402 + (abs((int)encSteps) * 2) % 120;
    int centerCh  = centerMhz - 2400;
    int rfHits = 0;
    for (int s = 0; s < 35; s++) {
      for (int off = -2; off <= 2; off++) {
        if (sampleNrfChannel(constrain(centerCh + off, 0, 125), 140)) rfHits++;
      }
    }
    int rfPercent = constrain((rfHits * 100) / 80, 0, 100);
    int fakeDbm   = map(rfPercent, 0, 100, -90, -25);
    int threshDbm = map(gPct, 0, 100, -90, -30);

    foxRssiHist[foxHead] = fakeDbm;
    foxHead = (foxHead + 1) % FOX_HIST;
    variometerBeep(fakeDbm, threshDbm);

    char badge[20];
    snprintf(badge, sizeof(badge), "%d MHz", centerMhz);
    drawHeader("[3/6] RAW RF FIELD PROBE (NRF)", rfPercent > 30 ? TFT_MAGENTA : 0x2008, badge);

    canvas.setCursor(6, 24);
    canvas.setTextColor(TFT_CYAN);
    canvas.printf("TUNED FREQ: %d MHz (+/-2MHz) | RF DENSITY: %3d%%", centerMhz, rfPercent);

    canvas.drawRect(6, 40, 308, 108, 0x2104);
    for (int i = 0; i < FOX_HIST - 1; i++) {
      int i1 = (foxHead + i) % FOX_HIST;
      int i2 = (foxHead + i + 1) % FOX_HIST;
      int x1 = 8 + (i * 302) / FOX_HIST;
      int x2 = 8 + ((i + 1) * 302) / FOX_HIST;
      int y1 = 146 - map(constrain((int)foxRssiHist[i1], -90, -25), -90, -25, 4, 100);
      int y2 = 146 - map(constrain((int)foxRssiHist[i2], -90, -25), -90, -25, 4, 100);
      canvas.drawLine(x1, y1, x2, y2, TFT_MAGENTA);
    }
    drawFooter(gPct, sub, 2, "Enc:Tune MHz | Finds Non-WiFi RF!");
  }
  canvas.pushSprite(0, 0);
  (void)zPct;
}

// ================= РЕЖИМ 4: WIDS SECURITY AUDITOR & HIDDEN CAM HUNTER =================
inline void runMode4_Security(int gPct, int zPct, int view, int32_t encSteps) {
  setSniffer(true);
  tickSnifferStats1Hz();

  static uint32_t lastHop = 0;
  static int ch = 1;
  if (millis() - lastHop > 150) {
    lastHop = millis();
    ch = (ch % 13) + 1;
    esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE);
  }

  int sub = view % 2;
  canvas.fillSprite(TFT_BLACK);

  if (sub == 0) {
    // V:1/2 — Детектор Deauth-глушилок и поддельных точек доступа (Evil-Twin)
    bool underAttack = (deauthPerSec >= 3) || evilTwinDetected;
    if (underAttack && soundEnabled) tone(BUZZ_PIN, ((millis() / 120) % 2) ? 2800 : 1200, 60);

    drawHeader("[4/6] WIDS SECURITY GUARD", underAttack ? TFT_RED : TFT_DARKGREEN,
               underAttack ? "ALERT! ATTACK!" : "AIR SECURE");

    canvas.setCursor(6, 22);
    canvas.setTextColor(deauthPerSec > 0 ? TFT_RED : TFT_GREEN);
    canvas.printf("DEAUTH FLOOD : %3u pkt/s (Total:%u)", deauthPerSec, cntDeauth);

    canvas.setCursor(6, 36);
    canvas.setTextColor(evilTwinDetected ? TFT_RED : TFT_CYAN);
    canvas.printf("HOME AP GUARD: %s (%s)", WIFI_SSID, evilTwinDetected ? "EVIL-TWIN SPOOF!" : "VERIFIED OK");

    canvas.setCursor(6, 50);
    canvas.setTextColor(TFT_YELLOW);
    if (cntDeauth > 0) {
      canvas.printf("LAST ATTACKER: %02X:%02X:%02X:%02X:%02X:%02X",
                    lastAttackerMac[0], lastAttackerMac[1], lastAttackerMac[2],
                    lastAttackerMac[3], lastAttackerMac[4], lastAttackerMac[5]);
    } else {
      canvas.print("LAST ATTACKER: NONE DETECTED");
    }

    // График истории атак за последние 120 секунд
    canvas.drawRect(6, 66, 308, 84, 0x2104);
    canvas.setCursor(12, 70);
    canvas.setTextColor(0x632C);
    canvas.print("120s DEAUTH / JAMMING TIMELINE");

    for (int i = 0; i < 119; i++) {
      int idx = (deauthHistHead + i) % 120;
      int h = map(constrain((int)deauthHistory[idx], 0, 50), 0, 50, 0, 76);
      int x = 10 + (i * 298) / 120;
      if (h > 0) canvas.fillRect(x, 148 - h, 2, h, TFT_RED);
      else       canvas.drawPixel(x, 148, TFT_DARKGREEN);
    }
    drawFooter(gPct, sub, 2, "EncBtn:Hidden Cam Hunter");

  } else {
    // V:2/2 — Охотник за скрытыми Wi-Fi камерами и видео-стримерами (фильтр по DATA PPS и OUI)
    drawHeader("[4/6] HIDDEN CAM / STREAM HUNTER", 0x300C, "DATA STREAMERS");

    int minPpsFilter = map(gPct, 0, 100, 0, 25);
    canvas.setCursor(4, 22);
    canvas.setTextColor(TFT_YELLOW);
    canvas.printf("ACTIVE STREAMERS (Filter >=%d pkt/s):", minPpsFilter);
    canvas.drawFastHLine(0, 32, 320, 0x4208);

    int row = 0;
    for (int i = 0; i < MAX_NODES && row < 6; i++) {
      if (nodeTable[i].pktCount == 0 || (millis() - nodeTable[i].lastSeen > 20000)) continue;
      bool isCamOui = (nodeTable[i].mac[0] == 0x24 && nodeTable[i].mac[1] == 0x6F) ||
                      (nodeTable[i].mac[0] == 0xA0 && nodeTable[i].mac[1] == 0x20) ||
                      (nodeTable[i].mac[0] == 0xBC && nodeTable[i].mac[1] == 0x51);
      if ((int)nodeTable[i].dataPps < minPpsFilter && !isCamOui && nodeTable[i].frameType != 2) continue;

      if (nodeTable[i].dataPps > 10) geigerTick(constrain(nodeTable[i].dataPps * 6, 40, 240));

      canvas.setCursor(4, 37 + row * 19);
      canvas.setTextColor(isCamOui || nodeTable[i].dataPps > 15 ? TFT_RED : TFT_CYAN);
      canvas.printf("%02X:%02X:%02X:%02X:%02X:%02X Ch%02d %3ddB %3upps %s",
                    nodeTable[i].mac[0], nodeTable[i].mac[1], nodeTable[i].mac[2],
                    nodeTable[i].mac[3], nodeTable[i].mac[4], nodeTable[i].mac[5],
                    nodeTable[i].channel, (int)nodeTable[i].smoothRssi,
                    nodeTable[i].dataPps, getVendorName(nodeTable[i].mac, nodeTable[i].frameType));
      row++;
    }
    if (row == 0) {
      canvas.setCursor(16, 82);
      canvas.setTextColor(TFT_GREEN);
      canvas.print("No High-Rate Video Streamers Detected.");
    }
    drawFooter(gPct, sub, 2, "Pot:PPS Filter | Finds IP-Cams");
  }
  canvas.pushSprite(0, 0);
  (void)zPct; (void)encSteps;
}

// ================= РЕЖИМ 5: 2.4GHz NON-WIFI PROTOCOL LAB (BLE / ZIGBEE / UPPER ISM) =================
inline void runMode5_ProtocolLab(int gPct, int zPct, int view) {
  // Выключаем Wi-Fi, чтобы NRF24 сканировал эфир со скоростью 100 кадров/с без собственных помех!
  setSniffer(false);

  int sub = view % 2;
  canvas.fillSprite(TFT_BLACK);

  if (sub == 0) {
    // V:1/2 — Монитор рекламных каналов Bluetooth Low Energy (BLE Ch 37=2402, Ch 38=2426, Ch 39=2480 МГц) + FHSS
    const uint8_t bleNrfCh[3] = {2, 26, 80};
    int hopBursts = 0;

    for (int k = 0; k < 3; k++) {
      int hits = 0;
      for (int s = 0; s < 25; s++) {
        if (sampleNrfChannel(bleNrfCh[k], 160)) hits++;
      }
      int lvl = constrain(hits * 12 * (1 + gPct / 30), 0, 255);
      bleAdvHits[k] = (bleAdvHits[k] * 2 + lvl) / 3;
      if (lvl > 40) hopBursts++;
    }

    geigerTick(max(bleAdvHits[0], max(bleAdvHits[1], bleAdvHits[2])));

    drawHeader("[5/6] BLE & ZIGBEE AIR MONITOR", 0x0210, "NRF24 RAW");

    const char* bleLabels[3] = {"BLE CH37 (2402MHz)", "BLE CH38 (2426MHz)", "BLE CH39 (2480MHz)"};
    for (int k = 0; k < 3; k++) {
      int y = 26 + k * 38;
      canvas.setCursor(8, y);
      canvas.setTextColor(TFT_CYAN);
      canvas.printf("%s : %3d%%", bleLabels[k], (bleAdvHits[k] * 100) / 255);

      canvas.drawRect(8, y + 12, 304, 16, 0x4208);
      int w = map(bleAdvHits[k], 0, 255, 2, 300);
      canvas.fillRect(10, y + 14, w, 12, bleAdvHits[k] > 140 ? TFT_MAGENTA : TFT_GREEN);
    }

    canvas.setCursor(8, 140);
    canvas.setTextColor(TFT_YELLOW);
    canvas.printf("DETECTED: %s", hopBursts >= 2 ? "ACTIVE BLE BEACONS / AIRTAGS / AUDIO" : "LOW BACKGROUND BLE");
    drawFooter(gPct, sub, 2, "EncBtn:Upper ISM (2480-2525MHz)");

  } else {
    // V:2/2 — Сканер верхнего ISM диапазона 2480..2525 МГц (выше Wi-Fi: беспроводные микрофоны, дроны, радиотелеметрия!)
    int maxUpper = 0, pkFreq = 2480;
    for (int i = 0; i < 46; i++) {
      uint8_t nrfCh = 80 + i; // 2480..2525 MHz
      int hits = 0;
      for (int s = 0; s < 6; s++) {
        if (sampleNrfChannel(nrfCh, 170)) hits++;
      }
      int val = constrain(hits * 45 * (1 + gPct / 35), 10, 255);
      upperIsmSpec[i] = (upperIsmSpec[i] + val) / 2;
      if (upperIsmSpec[i] > maxUpper) { maxUpper = upperIsmSpec[i]; pkFreq = 2480 + i; }
      upperIsmWf[upperWfRow][i] = upperIsmSpec[i];
    }
    upperWfRow = (upperWfRow + 1) % 70;
    geigerTick(maxUpper > 35 ? maxUpper : 0);

    char badge[20];
    snprintf(badge, sizeof(badge), "PK:%dMHz", pkFreq);
    drawHeader("[5/6] UPPER ISM 2480-2525 MHz", 0x300C, badge);

    for (int i = 0; i < 45; i++) {
      int x1 = 8 + i * 6;
      int x2 = 8 + (i + 1) * 6;
      int y1 = 72 - map(upperIsmSpec[i], 0, 255, 2, 48);
      int y2 = 72 - map(upperIsmSpec[i + 1], 0, 255, 2, 48);
      canvas.drawLine(x1, y1, x2, y2, TFT_YELLOW);
    }

    for (int y = 0; y < 70; y++) {
      int r = (upperWfRow - 1 - y + 70) % 70;
      for (int i = 0; i < 46; i++) {
        canvas.drawFastHLine(8 + i * 6, 78 + y, 6, waterfallColor(upperIsmWf[r][i]));
      }
    }
    drawFooter(gPct, sub, 2, "Above Wi-Fi: Drones/Mics/FHSS");
  }
  canvas.pushSprite(0, 0);
  (void)zPct;
}