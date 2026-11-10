#pragma once
#include <WiFi.h>
#include "esp_wifi.h"
#include "hardware.h"

void stopNetMonitor();

// Узел в радиоэфире 802.11
struct SniffedNode {
  uint8_t  mac[6];
  char     ssid[15];
  float    smoothRssi;
  int8_t   lastRssi;
  int8_t   peakRssi;
  uint8_t  channel;
  uint8_t  frameType;    // 0=Beacon(AP), 1=Probe(Phone), 2=Data(Stream), 3=Deauth
  uint16_t pktCount;
  uint16_t dataPps;      // Скорость передачи пакетов данных в секунду (для поиска скрытых камер!)
  uint16_t dataAccum;
  uint32_t lastSeen;
};

// Перехваченный поисковый запрос смартфона (Probe Request SSID)
struct ProbeEntry {
  uint8_t  phoneMac[6];
  char     probedSsid[17];
  int8_t   rssi;
  uint16_t count;
  uint32_t lastSeen;
};

#define MAX_NODES  24
#define MAX_PROBES 12

SniffedNode nodeTable[MAX_NODES];
ProbeEntry  probeLog[MAX_PROBES];

volatile uint16_t chPktHits[14] = {0};
volatile uint8_t  chLastType[14] = {0};
volatile int8_t   chPeakRssi[14] = {-100};
volatile uint32_t cntBeacons = 0, cntProbes = 0, cntData = 0, cntDeauth = 0;

// Метрики безопасности WIDS (Режим 4)
volatile uint16_t deauthPerSec = 0;
volatile uint16_t deauthAccum  = 0;
uint8_t  lastAttackerMac[6] = {0};
uint8_t  homeApOriginalMac[6] = {0};
bool     homeApMacLearned = false;
bool     evilTwinDetected = false;
uint8_t  deauthHistory[120] = {0};
int      deauthHistHead = 0;

bool snifferRunning = false;

inline const char* getVendorName(const uint8_t* m, uint8_t fType) {
  uint32_t oui = (m[0] << 16) | (m[1] << 8) | m[2];
  if (oui == 0x60601F || oui == 0x34D262) return "DJI DRONE";
  if (m[0] == 0x24 && m[1] == 0x6F)       return "ESP32/CAM";
  if (m[0] == 0xA0 && m[1] == 0x20)       return "ESP/TUYA";
  if (m[0] == 0xDC && m[1] == 0xA6)       return "RaspbPi";
  if (m[0] == 0x50 && m[1] == 0xFF)       return "Keenetic";
  if (m[0] == 0xF4 && m[1] == 0xF2)       return "TP-Link";
  if (m[0] == 0x04 && m[1] == 0xD9)       return "ASUS";
  if (m[0] == 0xBC && m[1] == 0x51)       return "IP-CAMERA";
  if (m[0] & 0x02) {
    if (fType == 1) return "PHONE(Probe)";
    if (fType == 2) return "MOBILE(Data)";
    return "PRIVATE MAC";
  }
  if (fType == 0) return "WIFI ROUTER";
  if (fType == 1) return "CLIENT(Scan)";
  return "ACTIVE STREAM";
}

void IRAM_ATTR wifiSnifferCb(void* buf, wifi_promiscuous_pkt_type_t type) {
  if (type == WIFI_PKT_MISC) return;
  const wifi_promiscuous_pkt_t* pkt = (wifi_promiscuous_pkt_t*)buf;
  const uint8_t* p = pkt->payload;
  int len = pkt->rx_ctrl.sig_len;
  if (len < 24) return;

  int ch = constrain((int)pkt->rx_ctrl.channel, 1, 13);
  int rssi = pkt->rx_ctrl.rssi;

  chPktHits[ch]++;
  if (rssi > chPeakRssi[ch]) chPeakRssi[ch] = rssi;

  uint8_t fc0 = p[0];
  uint8_t fType = (fc0 >> 2) & 0x03;
  uint8_t fSub  = (fc0 >> 4) & 0x0F;
  uint8_t tag = 2;

  const uint8_t* srcMac = &p[10];

  if (fType == 0 && fSub == 8) {
    cntBeacons++; tag = 0; chLastType[ch] = 1;
  } else if (fType == 0 && fSub == 4) {
    cntProbes++;  tag = 1; chLastType[ch] = 2;
  } else if (fType == 0 && (fSub == 12 || fSub == 10)) {
    // Обнаружен фрейм Deauthentication (0xC0) или Disassociation (0xA0)!
    cntDeauth++; deauthAccum++; tag = 3;
    memcpy(lastAttackerMac, srcMac, 6);
  } else if (fType == 2) {
    cntData++;    tag = 2; chLastType[ch] = 3;
  }

  int slot = -1, oldestSlot = 0;
  uint32_t oldestTime = 0xFFFFFFFF;

  for (int i = 0; i < MAX_NODES; i++) {
    if (nodeTable[i].pktCount > 0 && memcmp(nodeTable[i].mac, srcMac, 6) == 0) { slot = i; break; }
    if (nodeTable[i].lastSeen < oldestTime) { oldestTime = nodeTable[i].lastSeen; oldestSlot = i; }
  }
  if (slot == -1) {
    slot = oldestSlot;
    memcpy(nodeTable[slot].mac, srcMac, 6);
    nodeTable[slot].pktCount = 0;
    nodeTable[slot].dataPps = 0;
    nodeTable[slot].dataAccum = 0;
    nodeTable[slot].smoothRssi = rssi;
    nodeTable[slot].peakRssi = rssi;
    nodeTable[slot].ssid[0] = '\0';
  } else {
    nodeTable[slot].smoothRssi = nodeTable[slot].smoothRssi * 0.80f + rssi * 0.20f;
    if (rssi > nodeTable[slot].peakRssi) nodeTable[slot].peakRssi = rssi;
  }

  if (fType == 2) nodeTable[slot].dataAccum++;

  // Парсинг SSID из Beacon (36) и Probe Request (24)
  if (fType == 0 && (fSub == 8 || fSub == 4)) {
    int ieOffset = (fSub == 8) ? 36 : 24;
    if (len > ieOffset + 2 && p[ieOffset] == 0x00) {
      int ssidLen = p[ieOffset + 1];
      if (ssidLen > 0 && ssidLen <= 32 && ieOffset + 2 + ssidLen <= len) {
        char extracted[17];
        int copyLen = min(ssidLen, 16);
        int validChars = 0;
        for (int c = 0; c < copyLen; c++) {
          char chChar = (char)p[ieOffset + 2 + c];
          if (chChar >= 32 && chChar <= 126) extracted[validChars++] = chChar;
        }
        extracted[validChars] = '\0';

        if (validChars > 0) {
          strncpy(nodeTable[slot].ssid, extracted, 14);
          nodeTable[slot].ssid[14] = '\0';

          // Проверка на Evil-Twin (подмену домашней точки доступа)
          if (fSub == 8 && strcmp(extracted, WIFI_SSID) == 0) {
            if (!homeApMacLearned) {
              memcpy(homeApOriginalMac, srcMac, 6);
              homeApMacLearned = true;
            } else if (memcmp(homeApOriginalMac, srcMac, 4) != 0) {
              evilTwinDetected = true;
            }
          }

          // Если это Probe Request от смартфона — сохраняем в отдельный журнал искомых сетей!
          if (fSub == 4) {
            int pSlot = -1, oldPSlot = 0;
            uint32_t oldPTime = 0xFFFFFFFF;
            for (int k = 0; k < MAX_PROBES; k++) {
              if (probeLog[k].count > 0 && strcmp(probeLog[k].probedSsid, extracted) == 0) { pSlot = k; break; }
              if (probeLog[k].lastSeen < oldPTime) { oldPTime = probeLog[k].lastSeen; oldPSlot = k; }
            }
            if (pSlot == -1) {
              pSlot = oldPSlot;
              strncpy(probeLog[pSlot].probedSsid, extracted, 16);
              probeLog[pSlot].probedSsid[16] = '\0';
              probeLog[pSlot].count = 0;
            }
            memcpy(probeLog[pSlot].phoneMac, srcMac, 6);
            probeLog[pSlot].rssi = rssi;
            probeLog[pSlot].count++;
            probeLog[pSlot].lastSeen = millis();
          }
        }
      }
    }
  }

  nodeTable[slot].lastRssi = rssi;
  nodeTable[slot].channel = ch;
  if (tag != 0 || nodeTable[slot].pktCount == 0) nodeTable[slot].frameType = tag;
  nodeTable[slot].pktCount++;
  nodeTable[slot].lastSeen = millis();
}

inline void setSniffer(bool on) {
  if (on == snifferRunning) return;
  if (on) {
    stopNetMonitor();
    WiFi.mode(WIFI_STA);
    WiFi.disconnect(false, true);
    esp_wifi_set_promiscuous(true);
    esp_wifi_set_promiscuous_rx_cb(&wifiSnifferCb);
    snifferRunning = true;
  } else {
    esp_wifi_set_promiscuous(false);
    esp_wifi_set_promiscuous_rx_cb(nullptr);
    snifferRunning = false;
  }
}

// Ежесекундное обновление скорости потока данных (PPS) и счетчика Deauth-атак
inline void tickSnifferStats1Hz() {
  static uint32_t lastSecMs = 0;
  if (millis() - lastSecMs >= 1000) {
    lastSecMs = millis();
    deauthPerSec = deauthAccum;
    deauthAccum = 0;
    deauthHistory[deauthHistHead] = constrain(deauthPerSec, 0, 255);
    deauthHistHead = (deauthHistHead + 1) % 120;

    for (int i = 0; i < MAX_NODES; i++) {
      nodeTable[i].dataPps = nodeTable[i].dataAccum;
      nodeTable[i].dataAccum = 0;
    }
  }
}