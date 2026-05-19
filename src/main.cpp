#include <Arduino.h>
#include <Preferences.h>
#include <SPI.h>
#include <SD.h>
#include <WiFi.h>
#include <time.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <Adafruit_NeoPixel.h>
#include <ThreeWire.h>
#include <RtcDS1302.h>

#include "wifi_config.h"

namespace Pins {
constexpr int SPI_SCLK = 12;
constexpr int SPI_MOSI = 11;
constexpr int SPI_MISO = 13;

constexpr int TFT_CS = 10;
constexpr int TFT_DC = 9;
constexpr int TFT_RST = 14;
constexpr int TFT_BL = 21;

constexpr int SD_CS = 8;

constexpr int RS485_TX = 17;
constexpr int RS485_RX = 18;

constexpr int RTC_CLK = 4;
constexpr int RTC_DAT = 5;
constexpr int RTC_RST = 6;

constexpr int RGB_LED = 48;
}  // namespace Pins

constexpr uint8_t CO2_SLAVE_ADDR = 0x01;
constexpr uint16_t CO2_REGISTER = 0x0005;
constexpr uint32_t MODBUS_BAUD = 9600;
constexpr uint32_t WIFI_SYNC_WINDOW_MS = 30000;
constexpr uint32_t LOG_START_DELAY_MS = 60000;
constexpr uint32_t LOG_INTERVAL_MS = 10000;
constexpr char BUILD_ID[] = __DATE__ " " __TIME__;
constexpr char PREF_NAMESPACE[] = "co2-record";
constexpr char PREF_BUILD_KEY[] = "build-id";

SPIClass spi(FSPI);
Adafruit_ST7735 tft(&spi, Pins::TFT_CS, Pins::TFT_DC, Pins::TFT_RST);
HardwareSerial rs485(1);
Adafruit_NeoPixel statusLed(1, Pins::RGB_LED, NEO_GRB + NEO_KHZ800);
ThreeWire rtcWire(Pins::RTC_DAT, Pins::RTC_CLK, Pins::RTC_RST);
RtcDS1302<ThreeWire> rtc(rtcWire);
Preferences prefs;

bool sdReady = false;
bool rtcReady = false;
bool wifiSynced = false;
bool waitingForFirstLog = true;
uint32_t lastReadMs = 0;
uint32_t sampleNo = 0;
uint16_t lastCo2 = 0;
bool lastReadOk = false;
char lastTimeText[20] = "----/--/-- --:--:--";

struct Timestamp {
  uint16_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
};

void showLed(uint8_t red, uint8_t green, uint8_t blue) {
  statusLed.setPixelColor(0, statusLed.Color(red, green, blue));
  statusLed.show();
}

void blinkLed(uint8_t red, uint8_t green, uint8_t blue, uint8_t times,
              uint16_t onMs, uint16_t offMs) {
  for (uint8_t i = 0; i < times; ++i) {
    showLed(red, green, blue);
    delay(onMs);
    showLed(0, 0, 0);
    delay(offMs);
  }
}

bool hasWifiCredentials() {
  return strcmp(WifiConfig::SSID, "YOUR_WIFI_SSID") != 0 &&
         strlen(WifiConfig::SSID) > 0;
}

bool shouldSyncWifiForThisBuild() {
  if (!prefs.begin(PREF_NAMESPACE, false)) {
    Serial.println("NVS open failed, skip one-shot Wi-Fi check");
    return false;
  }

  const String savedBuildId = prefs.getString(PREF_BUILD_KEY, "");
  const bool shouldSync = savedBuildId != BUILD_ID;

  if (shouldSync) {
    prefs.putString(PREF_BUILD_KEY, BUILD_ID);
  }

  prefs.end();
  return shouldSync;
}

void formatTimestamp(const Timestamp &ts, char *buffer, size_t len) {
  snprintf(buffer, len, "%04u/%02u/%02u %02u:%02u:%02u", ts.year, ts.month,
           ts.day, ts.hour, ts.minute, ts.second);
}

bool readTimestamp(Timestamp &ts) {
  if (!rtcReady) {
    return false;
  }

  const RtcDateTime now = rtc.GetDateTime();
  if (!now.IsValid()) {
    return false;
  }

  ts.year = now.Year();
  ts.month = now.Month();
  ts.day = now.Day();
  ts.hour = now.Hour();
  ts.minute = now.Minute();
  ts.second = now.Second();
  return true;
}

bool setRtcFromNtp() {
  configTzTime(WifiConfig::TZ, "ntp.aliyun.com", "pool.ntp.org",
               "time.nist.gov");

  struct tm timeInfo = {};
  const uint32_t startMs = millis();
  while (millis() - startMs < 10000) {
    if (getLocalTime(&timeInfo, 1000)) {
      rtc.SetDateTime(RtcDateTime(timeInfo.tm_year + 1900, timeInfo.tm_mon + 1,
                                  timeInfo.tm_mday, timeInfo.tm_hour,
                                  timeInfo.tm_min, timeInfo.tm_sec));
      return true;
    }
  }

  return false;
}

void syncRtcFromWifi() {
  if (!hasWifiCredentials()) {
    Serial.println("Wi-Fi skipped: credentials not configured");
    return;
  }

  WiFi.mode(WIFI_STA);
  WiFi.begin(WifiConfig::SSID, WifiConfig::PASSWORD);
  Serial.print("Wi-Fi connect");

  const uint32_t startMs = millis();
  while (WiFi.status() != WL_CONNECTED &&
         millis() - startMs < WIFI_SYNC_WINDOW_MS) {
    Serial.print('.');
    delay(500);
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED && setRtcFromNtp()) {
    wifiSynced = true;
    Serial.println("Wi-Fi time sync OK");
    blinkLed(0, 0, 48, 3, 180, 120);
  } else {
    Serial.println("Wi-Fi time sync failed");
  }

  WiFi.disconnect(true, true);
  WiFi.mode(WIFI_OFF);
}

uint16_t modbusCrc16(const uint8_t *data, size_t len) {
  uint16_t crc = 0xFFFF;

  for (size_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8; ++bit) {
      if (crc & 0x0001) {
        crc = (crc >> 1) ^ 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }

  return crc;
}

void clearRs485Rx() {
  while (rs485.available()) {
    rs485.read();
  }
}

bool readHoldingRegister(uint8_t slaveAddr, uint16_t regAddr, uint16_t &value) {
  uint8_t request[8] = {
      slaveAddr,
      0x03,
      static_cast<uint8_t>(regAddr >> 8),
      static_cast<uint8_t>(regAddr & 0xFF),
      0x00,
      0x01,
      0x00,
      0x00,
  };

  const uint16_t reqCrc = modbusCrc16(request, 6);
  request[6] = static_cast<uint8_t>(reqCrc & 0xFF);
  request[7] = static_cast<uint8_t>(reqCrc >> 8);

  clearRs485Rx();
  rs485.write(request, sizeof(request));
  rs485.flush();

  uint8_t response[7] = {};
  size_t received = 0;
  const uint32_t startMs = millis();

  while (received < sizeof(response) && millis() - startMs < 1000) {
    if (rs485.available()) {
      response[received++] = static_cast<uint8_t>(rs485.read());
    }
    delay(1);
  }

  if (received != sizeof(response)) {
    return false;
  }

  const uint16_t respCrc = static_cast<uint16_t>(response[5]) |
                           (static_cast<uint16_t>(response[6]) << 8);
  if (modbusCrc16(response, 5) != respCrc) {
    return false;
  }

  if (response[0] != slaveAddr || response[1] != 0x03 || response[2] != 0x02) {
    return false;
  }

  value = (static_cast<uint16_t>(response[3]) << 8) | response[4];
  return true;
}

void drawStaticUi() {
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextWrap(false);

  tft.setTextColor(ST77XX_CYAN);
  tft.setTextSize(1);
  tft.setCursor(8, 8);
  tft.print("TAS CO2 LOGGER");

  tft.drawFastHLine(0, 24, tft.width(), ST77XX_BLUE);
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(8, 96);
  tft.print("TIME:");
  tft.setCursor(8, 112);
  tft.print("SD:");
}

void drawSdStatus() {
  tft.fillRect(30, 108, 90, 16, ST77XX_BLACK);
  tft.setCursor(32, 112);
  tft.setTextSize(1);
  tft.setTextColor(sdReady ? ST77XX_GREEN : ST77XX_RED);
  tft.print(sdReady ? "READY" : "FAILED");
}

void drawTimeStatus(const char *timeText) {
  tft.fillRect(44, 92, 84, 12, ST77XX_BLACK);
  tft.setCursor(44, 96);
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_YELLOW);
  tft.print(timeText);
}

void drawCo2(uint16_t ppm, bool ok) {
  tft.fillRect(0, 32, tft.width(), 72, ST77XX_BLACK);

  if (ok) {
    tft.setTextColor(ST77XX_WHITE);
    tft.setTextSize(2);
    tft.setCursor(8, 36);
    tft.print("CO2");

    tft.setTextColor(ST77XX_GREEN);
    tft.setTextSize(3);
    tft.setCursor(8, 62);
    tft.print(ppm);

    tft.setTextColor(ST77XX_WHITE);
    tft.setTextSize(1);
    tft.setCursor(98, 82);
    tft.print("ppm");
  } else {
    tft.setTextColor(ST77XX_RED);
    tft.setTextSize(2);
    tft.setCursor(8, 52);
    tft.print("READ FAIL");
  }
}

void appendLog(const char *timeText, uint16_t ppm, bool ok) {
  if (!sdReady) {
    return;
  }

  File logFile = SD.open("/co2_log.csv", FILE_APPEND);
  if (!logFile) {
    sdReady = false;
    drawSdStatus();
    return;
  }

  logFile.print(sampleNo);
  logFile.print(',');
  logFile.print(timeText);
  logFile.print(',');
  if (ok) {
    logFile.println(ppm);
  } else {
    logFile.println("ERR");
  }
  logFile.close();
}

void setupSdCard() {
  sdReady = SD.begin(Pins::SD_CS, spi);
  drawSdStatus();

  if (!sdReady) {
    Serial.println("SD mount failed");
    return;
  }

  if (!SD.exists("/co2_log.csv")) {
    File logFile = SD.open("/co2_log.csv", FILE_WRITE);
    if (logFile) {
      logFile.println("sample,time,co2_ppm");
      logFile.close();
    }
  }
}

void setupRtc() {
  rtc.Begin();
  rtc.SetIsWriteProtected(false);
  rtc.SetIsRunning(true);

  const RtcDateTime now = rtc.GetDateTime();
  rtcReady = now.IsValid();
  if (!rtcReady) {
    rtc.SetDateTime(RtcDateTime(__DATE__, __TIME__));
    rtcReady = true;
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);

  statusLed.begin();
  showLed(48, 0, 0);

  pinMode(Pins::TFT_BL, OUTPUT);
  digitalWrite(Pins::TFT_BL, HIGH);

  spi.begin(Pins::SPI_SCLK, Pins::SPI_MISO, Pins::SPI_MOSI);

  tft.initR(INITR_BLACKTAB);
  tft.setRotation(1);
  drawStaticUi();

  setupRtc();
  if (shouldSyncWifiForThisBuild()) {
    Serial.println("First boot for this firmware, trying Wi-Fi time sync");
    syncRtcFromWifi();
  } else {
    Serial.println("Wi-Fi sync skipped: already handled for this firmware");
  }
  setupSdCard();

  rs485.begin(MODBUS_BAUD, SERIAL_8N1, Pins::RS485_RX, Pins::RS485_TX);

  Serial.println("ESP32-S3 CO2 logger ready");
  Serial.println("RS485: 9600 8N1, slave 0x01, register 0x0005");
}

void loop() {
  Timestamp ts = {};
  if (readTimestamp(ts)) {
    formatTimestamp(ts, lastTimeText, sizeof(lastTimeText));
  }
  drawTimeStatus(lastTimeText);

  const uint32_t now = millis();
  if (now < LOG_START_DELAY_MS) {
    if (waitingForFirstLog) {
      showLed(48, 0, 0);
    }
    return;
  }

  if (now - lastReadMs < LOG_INTERVAL_MS) {
    return;
  }
  lastReadMs = now;
  ++sampleNo;

  uint16_t co2 = 0;
  const bool ok = readHoldingRegister(CO2_SLAVE_ADDR, CO2_REGISTER, co2);

  if (ok) {
    lastCo2 = co2;
  }
  lastReadOk = ok;
  waitingForFirstLog = false;

  drawCo2(lastCo2, lastReadOk);
  appendLog(lastTimeText, lastCo2, lastReadOk);
  blinkLed(0, 48, 0, 1, 120, 0);

  Serial.print("CO2 read ");
  Serial.print(ok ? "OK: " : "FAIL, last: ");
  Serial.print(lastCo2);
  Serial.print(" ppm @ ");
  Serial.print(lastTimeText);
  Serial.println(" ppm");
}
