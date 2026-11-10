#pragma once
#include <Arduino.h>

// ================= НАСТРОЙКИ WI-FI ДЛЯ РЕЖИМА 6 =================
#define WIFI_SSID "onceupon2.4Ghz"
#define WIFI_PASS "olegshmitov"

// ================= РАСПИНОВКА УСТРОЙСТВА =================
// Дисплей 1.9" IPS ST7789 (170x320)
#define TFT_SCLK 18
#define TFT_MOSI 23
#define TFT_RST   4
#define TFT_DC    2
#define TFT_CS   15

// Радиомодуль NRF24L01+PA+LNA
#define NRF_CE   27
#define NRF_CSN  26
#define NRF_SCK  14
#define NRF_MOSI 13
#define NRF_MISO 12

// Звук и органы управления
#define BUZZ_PIN 25
#define POT_GAIN 34   // Главный потенциометр (Gain / Threshold / Sensitivity)
#define POT_ZOOM 35   // Второй потенциометр (Squelch / Probe Speed — если подключен)
#define ENC_S1   32   // Энкодер CLK
#define ENC_S2   33   // Энкодер DT
#define ENC_KEY  19   // Кнопка энкодера (переключение вьюшек V:1 / V:2 / V:3)
#define BTN_MODE  5   // Кнопка переключения главных режимов (1..6)

#define TOTAL_MODES 6