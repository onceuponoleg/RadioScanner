#pragma once
#include <WiFi.h>
#include <lwip/sockets.h>
#include <fcntl.h>
#include <errno.h>
#include "hardware.h"
#include "sniffer_engine.h"

const IPAddress WAN_HOSTS[3] = {
  IPAddress(1, 1, 1, 1),     // Cloudflare Anycast
  IPAddress(8, 8, 8, 8),     // Google Public Anycast
  IPAddress(77, 88, 8, 8)    // Yandex Anycast
};
const char* WAN_HOST_NAMES[3] = {"1.1.1.1:443", "8.8.8.8:443", "77.88.8.8:443"};
int wanTargetIdx = 0;

#define NET_HIST 180
uint16_t wanRttHist[NET_HIST];
uint16_t gwRttHist[NET_HIST];
uint8_t  lossHist[NET_HIST];
int netHead = 0;

uint16_t grid100Rtt[100];
bool     grid100Lost[100];
int      grid100Idx = 0;

#define NET_WF_H 28
uint8_t netWaterfall[NET_WF_H][90];
int netWfRow = 0;

float curWanRtt = 0.0f;
float curGwRtt  = 2.0f;
float curJitter = 0.0f;
float minWanRtt = 999.0f, maxWanRtt = 0.0f;
float packetLossPct = 0.0f;
int   netQualityScore = 100;
float mosScore = 4.4f;
int   consecutiveLosses = 0;

int      wanSock = -1;
bool     wanInFlight = false;
uint32_t wanSentUs = 0;

int      gwSock = -1;
bool     gwInFlight = false;
uint32_t gwSentUs = 0;

uint32_t lastProbeMs = 0;
uint32_t lastWifiAttemptMs = 0;
uint32_t wifiReadyMs = 0;
bool     wifiStaActive = false;
bool     wasConnected = false;

inline void closeSocketImmediate(int &s) {
  if (s >= 0) {
    struct linger sl = {1, 0};
    setsockopt(s, SOL_SOCKET, SO_LINGER, &sl, sizeof(sl));
    close(s);
    s = -1;
  }
}

inline void stopNetMonitor() {
  closeSocketImmediate(wanSock);
  closeSocketImmediate(gwSock);
  wanInFlight = false;
  gwInFlight = false;
  if (wifiStaActive) {
    WiFi.disconnect(false, true);
    wifiStaActive = false;
    wasConnected = false;
  }
}

inline void resetNetStats() {
  memset(wanRttHist, 0, sizeof(wanRttHist));
  memset(gwRttHist, 0, sizeof(gwRttHist));
  memset(lossHist, 0, sizeof(lossHist));
  memset(grid100Rtt, 0, sizeof(grid100Rtt));
  memset(grid100Lost, 0, sizeof(grid100Lost));
  minWanRtt = 999.0f;
  maxWanRtt = 0.0f;
  curJitter = 0.0f;
  packetLossPct = 0.0f;
  consecutiveLosses = 0;
  netQualityScore = 100;
}

inline bool startTcpProbe(IPAddress targetIp, uint16_t port, int &sock, uint32_t &sentUs, bool &inFlight) {
  closeSocketImmediate(sock);
  inFlight = false;

  sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
  if (sock < 0) return false;

  int flags = fcntl(sock, F_GETFL, 0);
  fcntl(sock, F_SETFL, flags | O_NONBLOCK);

  struct sockaddr_in addr;
  memset(&addr, 0, sizeof(addr));
  addr.sin_family = AF_INET;
  addr.sin_port = htons(port);
  addr.sin_addr.s_addr = (uint32_t)targetIp;

  sentUs = micros();
  int res = connect(sock, (struct sockaddr*)&addr, sizeof(addr));
  if (res == 0 || errno == EINPROGRESS || errno == EALREADY) {
    inFlight = true;
    return true;
  }

  closeSocketImmediate(sock);
  return false;
}

inline bool pollTcpProbe(int &sock, uint32_t sentUs, bool &inFlight, bool allowRefused,
                         uint16_t &outRttMs, bool &outLost, uint32_t timeoutUs) {
  if (!inFlight || sock < 0) return false;

  fd_set rfds, wfds, efds;
  FD_ZERO(&rfds); FD_ZERO(&wfds); FD_ZERO(&efds);
  FD_SET(sock, &rfds); FD_SET(sock, &wfds); FD_SET(sock, &efds);
  struct timeval tv = {0, 0};

  int sel = select(sock + 1, &rfds, &wfds, &efds, &tv);
  if (sel > 0) {
    int soErr = 0;
    socklen_t len = sizeof(soErr);
    getsockopt(sock, SOL_SOCKET, SO_ERROR, &soErr, &len);

    uint32_t elapsedUs = micros() - sentUs;
    closeSocketImmediate(sock);
    inFlight = false;

    if (soErr == 0 || (allowRefused && (soErr == ECONNREFUSED || soErr == 111 || soErr == 104))) {
      outRttMs = constrain((int)(elapsedUs / 1000), 1, 999);
      outLost = false;
    } else {
      outRttMs = 999;
      outLost = true;
    }
    return true;
  }

  if (micros() - sentUs > timeoutUs) {
    closeSocketImmediate(sock);
    inFlight = false;
    outRttMs = 999;
    outLost = true;
    return true;
  }

  return false;
}

inline void recordNetSample(uint16_t wanMs, uint16_t gwMs, bool lost, int lagThresholdMs) {
  if (!lost) {
    consecutiveLosses = 0;
    if (curWanRtt > 0.1f) {
      float diff = fabsf((float)wanMs - curWanRtt);
      curJitter = curJitter * 0.80f + diff * 0.20f;
    }
    curWanRtt = (curWanRtt < 0.1f) ? wanMs : (curWanRtt * 0.6f + wanMs * 0.4f);
    if (wanMs < minWanRtt) minWanRtt = wanMs;
    if (wanMs > maxWanRtt) maxWanRtt = wanMs;

    if ((int)wanMs > lagThresholdMs || (int)(curJitter * 2.5f) > lagThresholdMs) {
      int lagSeverity = constrain((int)(wanMs - lagThresholdMs) * 3 + (int)(curJitter * 4), 45, 250);
      geigerTick(lagSeverity);
    }
  } else {
    consecutiveLosses++;
    if (soundEnabled) tone(BUZZ_PIN, 520, 45);
  }

  wanRttHist[netHead] = lost ? 400 : wanMs;
  gwRttHist[netHead]  = gwMs;
  lossHist[netHead]   = lost ? 1 : 0;
  netHead = (netHead + 1) % NET_HIST;

  grid100Rtt[grid100Idx]  = lost ? 0 : wanMs;
  grid100Lost[grid100Idx] = lost;
  grid100Idx = (grid100Idx + 1) % 100;

  int lostCount = 0, validSamples = 0;
  for (int i = 0; i < 100; i++) {
    if (grid100Rtt[i] > 0 || grid100Lost[i]) {
      validSamples++;
      if (grid100Lost[i]) lostCount++;
    }
  }
  packetLossPct = (validSamples > 0) ? ((lostCount * 100.0f) / validSamples) : 0.0f;

  float effectiveLatency = curWanRtt + curJitter * 2.0f + 10.0f;
  float rFactor = 93.2f;
  if (effectiveLatency < 160.0f) rFactor -= (effectiveLatency / 40.0f);
  else                           rFactor -= ((effectiveLatency - 120.0f) / 10.0f);
  rFactor -= (packetLossPct * 12.5f);
  if (consecutiveLosses >= 2) rFactor -= 35.0f;

  netQualityScore = constrain((int)rFactor, 0, 100);
  if (rFactor < 6.5f) mosScore = 1.0f;
  else mosScore = constrain(1.0f + 0.035f * rFactor + rFactor * (rFactor - 60.0f) * (100.0f - rFactor) * 0.000007f, 1.0f, 4.5f);

  for (int x = 0; x < 90; x++) {
    int h = (netHead - 90 + x + NET_HIST) % NET_HIST;
    if (lossHist[h]) netWaterfall[netWfRow][x] = 255;
    else netWaterfall[netWfRow][x] = constrain(map((int)wanRttHist[h], 5, 220, 35, 225), 25, 230);
  }
  netWfRow = (netWfRow + 1) % NET_WF_H;
}

// ================= РЕЖИМ 6: REALTIME INTERNET QUALITY MONITOR =================
inline void runMode6_NetQuality(int gPct, int zPct, int view) {
  setSniffer(false);

  if (!wifiStaActive) {
    WiFi.disconnect(true, true);
    delay(40);
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    wifiStaActive = true;
    wasConnected = false;
    lastWifiAttemptMs = millis();
  }

  bool isOnline = (WiFi.status() == WL_CONNECTED) && (WiFi.localIP()[0] != 0) && (WiFi.gatewayIP()[0] != 0);

  if (!isOnline) {
    wasConnected = false;
    if (millis() - lastWifiAttemptMs > 9000) {
      WiFi.disconnect(true, true);
      delay(20);
      WiFi.begin(WIFI_SSID, WIFI_PASS);
      lastWifiAttemptMs = millis();
    }
    canvas.fillSprite(TFT_BLACK);
    drawHeader("[6/6] REALTIME NET QUALITY", TFT_NAVY, "CONNECTING");
    canvas.setCursor(14, 42);
    canvas.setTextColor(TFT_CYAN);
    canvas.printf("SSID: %s", WIFI_SSID);
    canvas.setCursor(14, 64);
    canvas.setTextColor(TFT_YELLOW);
    canvas.print("Waiting for DHCP & Gateway ARP...");
    int barW = ((millis() / 15) % 260);
    canvas.drawRect(14, 90, 280, 12, TFT_WHITE);
    canvas.fillRect(16, 92, barW, 8, TFT_GREEN);
    drawFooter(gPct, view, 3, "Set WIFI_SSID in config.h");
    canvas.pushSprite(0, 0);
    return;
  }

  if (!wasConnected) {
    wasConnected = true;
    wifiReadyMs = millis();
    resetNetStats();
  }

  int lagAlarmMs   = map(gPct, 0, 100, 180, 20);
  uint32_t probeMs = map(zPct, 0, 100, 500, 150);

  uint16_t gwRtt = 0;
  bool gwLost = false;
  if (pollTcpProbe(gwSock, gwSentUs, gwInFlight, true, gwRtt, gwLost, 600000)) {
    if (!gwLost) curGwRtt = (curGwRtt < 0.1f) ? gwRtt : (curGwRtt * 0.6f + gwRtt * 0.4f);
    else         curGwRtt = 180.0f;
  }

  uint16_t wanRtt = 0;
  bool wanLost = false;
  if (pollTcpProbe(wanSock, wanSentUs, wanInFlight, true, wanRtt, wanLost, 1200000)) {
    recordNetSample(wanRtt, (uint16_t)curGwRtt, wanLost, lagAlarmMs);
  }

  if (!wanInFlight && (millis() - wifiReadyMs > 400) && (millis() - lastProbeMs >= probeMs)) {
    lastProbeMs = millis();
    startTcpProbe(WAN_HOSTS[wanTargetIdx], 443, wanSock, wanSentUs, wanInFlight);
    if (!gwInFlight) {
      startTcpProbe(WiFi.gatewayIP(), 80, gwSock, gwSentUs, gwInFlight);
    }
  }

  const char* verdictTxt = "IDEAL [CALL/GAME OK]";
  uint16_t verdictCol = TFT_DARKGREEN;
  if (consecutiveLosses >= 2 || packetLossPct >= 5.0f) {
    verdictTxt = "PACKET LOSS! [DROP]";
    verdictCol = TFT_RED;
  } else if (curGwRtt > 35.0f || WiFi.RSSI() < -80) {
    verdictTxt = "WIFI/ROUTER LAG!";
    verdictCol = TFT_ORANGE;
  } else if (curJitter > 18.0f || curWanRtt > lagAlarmMs) {
    verdictTxt = "ISP JITTER / LAG!";
    verdictCol = TFT_MAGENTA;
  }

  canvas.fillSprite(TFT_BLACK);
  char badge[20];
  snprintf(badge, sizeof(badge), "SCORE:%d%%", netQualityScore);
  drawHeader(verdictTxt, verdictCol, badge);

  int sub = view % 3;

  if (sub == 0) {
    canvas.setCursor(4, 21);   canvas.setTextColor(TFT_CYAN);   canvas.printf("WAN:%3.0fms", curWanRtt);
    canvas.setCursor(68, 21);  canvas.setTextColor(TFT_GREEN);  canvas.printf("GW:%2.0fms", curGwRtt);
    canvas.setCursor(120, 21); canvas.setTextColor(curJitter > 15.0f ? TFT_MAGENTA : TFT_YELLOW); canvas.printf("JIT:%2.0fms", curJitter);
    canvas.setCursor(178, 21); canvas.setTextColor(packetLossPct > 0.5f ? TFT_RED : TFT_WHITE);   canvas.printf("LOSS:%.0f%%", packetLossPct);
    canvas.setCursor(236, 21); canvas.setTextColor(TFT_LIGHTGREY); canvas.print(WAN_HOST_NAMES[wanTargetIdx]);

    canvas.drawRect(4, 31, 312, 86, 0x2104);
    int maxScaleMs = max(lagAlarmMs + 40, 100);
    int alarmY = 115 - constrain(map(lagAlarmMs, 0, maxScaleMs, 2, 82), 2, 82);
    for (int x = 8; x < 310; x += 6) canvas.drawFastHLine(x, alarmY, 3, TFT_RED);

    for (int i = 0; i < NET_HIST - 1; i++) {
      int i1 = (netHead + i) % NET_HIST;
      int i2 = (netHead + i + 1) % NET_HIST;
      int x1 = 6 + (i * 306) / NET_HIST;
      int x2 = 6 + ((i + 1) * 306) / NET_HIST;

      if (lossHist[i1]) {
        canvas.drawFastVLine(x1, 33, 82, TFT_RED);
        continue;
      }

      int wy1 = 115 - constrain(map((int)wanRttHist[i1], 0, maxScaleMs, 2, 82), 2, 82);
      int wy2 = 115 - constrain(map((int)wanRttHist[i2], 0, maxScaleMs, 2, 82), 2, 82);
      int gy1 = 115 - constrain(map((int)gwRttHist[i1],  0, maxScaleMs, 2, 82), 2, 82);
      int gy2 = 115 - constrain(map((int)gwRttHist[i2],  0, maxScaleMs, 2, 82), 2, 82);

      canvas.drawLine(x1, gy1, x2, gy2, TFT_GREEN);
      canvas.drawLine(x1, wy1, x2, wy2, (wanRttHist[i2] > lagAlarmMs) ? TFT_YELLOW : TFT_CYAN);
    }

    canvas.drawRect(4, 120, 312, 32, 0x2104);
    for (int y = 0; y < NET_WF_H; y++) {
      int r = (netWfRow - 1 - y + NET_WF_H) % NET_WF_H;
      for (int x = 0; x < 90; x++) {
        uint8_t v = netWaterfall[r][x];
        canvas.drawFastHLine(8 + x * 3, 122 + y, 3, (v == 255) ? TFT_RED : waterfallColor(v));
      }
    }
    drawFooter(gPct, sub, 3, "Enc:Server | EncBtn:Phase");

  } else if (sub == 1) {
    int bx = 6, by = 22, bw = 126, bh = 126;
    canvas.drawRect(bx, by, bw, bh, 0x4208);
    canvas.drawLine(bx + 2, by + bh - 2, bx + bw - 2, by + 2, 0x2104);

    int maxPlotMs = max(lagAlarmMs + 40, 120);
    for (int i = 1; i < NET_HIST; i++) {
      int idxPrev = (netHead + i - 1) % NET_HIST;
      int idxCur  = (netHead + i) % NET_HIST;
      if (wanRttHist[idxCur] == 0 && !lossHist[idxCur]) continue;

      if (lossHist[idxCur]) {
        canvas.fillCircle(bx + bw - 8, by + 8, 3, TFT_RED);
      } else {
        int px = bx + 4 + constrain(map((int)wanRttHist[idxPrev], 0, maxPlotMs, 0, bw - 8), 0, bw - 8);
        int py = by + bh - 4 - constrain(map((int)wanRttHist[idxCur], 0, maxPlotMs, 0, bh - 8), 0, bh - 8);
        bool isRecent = (i > NET_HIST - 15);
        canvas.fillCircle(px, py, isRecent ? 2 : 1, isRecent ? TFT_YELLOW : TFT_CYAN);
      }
    }
    canvas.setCursor(10, 25); canvas.setTextColor(TFT_LIGHTGREY); canvas.print("JITTER PHASE PLOT");

    canvas.drawRect(138, 22, 178, 126, 0x4208);
    canvas.setCursor(144, 27); canvas.setTextColor(TFT_YELLOW);
    canvas.printf("MOS CALL QUALITY: %.2f/4.5", mosScore);

    bool callsOk   = (mosScore >= 3.8f && packetLossPct < 1.5f && curJitter < 20.0f);
    bool gamesOk   = (curWanRtt <= lagAlarmMs && curJitter < 10.0f && packetLossPct < 0.5f);
    bool wifiFault = (curGwRtt > 25.0f || WiFi.RSSI() < -78);
    bool ispFault  = (!wifiFault && (curWanRtt > lagAlarmMs || packetLossPct >= 1.0f || curJitter > 15.0f));

    canvas.setCursor(144, 46); canvas.setTextColor(TFT_WHITE); canvas.print("ZOOM/VOICE : ");
    canvas.setTextColor(callsOk ? TFT_GREEN : TFT_RED);
    canvas.print(callsOk ? "CRYSTAL CLEAR" : "ROBOTIC / LAG");

    canvas.setCursor(144, 64); canvas.setTextColor(TFT_WHITE); canvas.print("GAMING/FPS : ");
    canvas.setTextColor(gamesOk ? TFT_GREEN : TFT_ORANGE);
    canvas.print(gamesOk ? "COMPETITIVE" : "RUBBERBANDING");

    canvas.setCursor(144, 82); canvas.setTextColor(TFT_WHITE); canvas.print("BOTTLENECK : ");
    if (wifiFault)     { canvas.setTextColor(TFT_ORANGE);  canvas.print("HOME WIFI/ROUTER"); }
    else if (ispFault) { canvas.setTextColor(TFT_MAGENTA); canvas.print("ISP EXTERNAL LINE"); }
    else               { canvas.setTextColor(TFT_GREEN);   canvas.print("NONE (CLEAN PATH)"); }

    canvas.drawFastHLine(142, 98, 170, 0x2104);
    canvas.setCursor(144, 105); canvas.setTextColor(TFT_CYAN);      canvas.printf("ISP BACKBONE: %3.0f ms", max(0.0f, curWanRtt - curGwRtt));
    canvas.setCursor(144, 120); canvas.setTextColor(TFT_GREEN);     canvas.printf("WIFI AIR RTT: %3.0f ms", curGwRtt);
    canvas.setCursor(144, 134); canvas.setTextColor(TFT_LIGHTGREY); canvas.print("PROBE: TCP SYN:443 (<0.2KB/s)");

    drawFooter(gPct, sub, 3, "Pot:LagLimit | EncBtn:Matrix");

  } else {
    canvas.setCursor(6, 22); canvas.setTextColor(TFT_YELLOW); canvas.print("LAST 100 PACKETS:");

    for (int r = 0; r < 10; r++) {
      for (int c = 0; c < 10; c++) {
        int idx = r * 10 + c;
        int sx = 6 + c * 11, sy = 34 + r * 11;
        uint16_t cellCol = 0x1082;
        if (grid100Lost[idx]) {
          cellCol = TFT_RED;
        } else if (grid100Rtt[idx] > 0) {
          uint16_t ms = grid100Rtt[idx];
          if (ms <= 45)                cellCol = TFT_GREEN;
          else if (ms <= lagAlarmMs)   cellCol = TFT_CYAN;
          else if (ms <= lagAlarmMs*2) cellCol = TFT_YELLOW;
          else                         cellCol = TFT_MAGENTA;
        }
        canvas.fillRect(sx, sy, 9, 9, cellCol);
        if (idx == (grid100Idx + 99) % 100) canvas.drawRect(sx - 1, sy - 1, 11, 11, TFT_WHITE);
      }
    }

    canvas.drawRect(122, 22, 194, 126, 0x4208);
    canvas.setCursor(128, 27); canvas.setTextColor(TFT_CYAN);   canvas.printf("WIFI : %.10s (%ddBm)", WiFi.SSID().c_str(), WiFi.RSSI());
    canvas.setCursor(128, 42); canvas.setTextColor(TFT_WHITE);  canvas.printf("MY IP: %s", WiFi.localIP().toString().c_str());
    canvas.setCursor(128, 57); canvas.setTextColor(TFT_GREEN);  canvas.printf("GW IP: %s (%.0fms)", WiFi.gatewayIP().toString().c_str(), curGwRtt);
    canvas.setCursor(128, 72); canvas.setTextColor(TFT_YELLOW); canvas.printf("HOST : %s", WAN_HOST_NAMES[wanTargetIdx]);

    canvas.drawFastHLine(126, 85, 186, 0x2104);
    canvas.setCursor(128, 92);  canvas.setTextColor(TFT_WHITE); canvas.printf("MIN / AVG / MAX : %.0f / %.0f / %.0fms", minWanRtt > 900 ? 0 : minWanRtt, curWanRtt, maxWanRtt);
    canvas.setCursor(128, 108); canvas.setTextColor(curJitter > 12.0f ? TFT_MAGENTA : TFT_CYAN); canvas.printf("JITTER (RFC3550): %.1f ms", curJitter);
    canvas.setCursor(128, 124); canvas.setTextColor(packetLossPct > 0.0f ? TFT_RED : TFT_GREEN); canvas.printf("PACKET LOSS 100 : %.1f %%", packetLossPct);

    drawFooter(gPct, sub, 3, "HoldEnc:Reset | TurnEnc:Host");
  }

  canvas.pushSprite(0, 0);
}