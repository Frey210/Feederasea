// firmware/esp8266/feeder_esp8266.ino
#include <Arduino.h>
#include "../../include/secrets.h"
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include <time.h>

namespace Config {
static const uint32_t UART_BAUD = 9600;
static const uint32_t SAMPLE_INTERVAL_MS = 2000;
static const uint32_t BLYNK_CONNECT_TIMEOUT_MS = 5000;
static const uint32_t WIFI_TIMEOUT_MS = 15000;
static const int SLOT_MORNING_H = 7;
static const int SLOT_EVENING_H = 17;
}  // namespace Config

namespace VPin {
static const uint8_t SIM_TEMP = V1;
static const uint8_t BIOMASS = V2;
static const uint8_t MANUAL_FEED = V3;
static const uint8_t MODE_SELECT = V4;
static const uint8_t PWM_PERCENT = V5;
static const uint8_t GPS_100 = V6;
static const uint8_t SIM_EVENT = V7;
static const uint8_t TEST_IN = V8;

static const uint8_t TEMP_C = V20;
static const uint8_t FEED_REMAIN = V21;
static const uint8_t BIOMASS_OUT = V22;
static const uint8_t LAST_CMD_GRAMS = V23;
static const uint8_t LAST_PWM = V24;
static const uint8_t LAST_EVENT = V25;
}  // namespace VPin

#define VPIN_SIM_TEMP V1
#define VPIN_BIOMASS V2
#define VPIN_MANUAL_FEED V3
#define VPIN_MODE_SELECT V4
#define VPIN_PWM_PERCENT V5
#define VPIN_GPS_100 V6
#define VPIN_SIM_EVENT V7
#define VPIN_TEST_IN V8

BlynkTimer timer;

struct AppInputs {
  bool simMode = false;
  float simTempC = 28.0f;
  float simDistanceCm = 25.0f;
  double biomassG = 0.0;
  int modeSelect = 0;
  int pwmPercent = 50;
  int gramsPerSec100 = 5;
};

static AppInputs inputs;

String serialLine;
int lastSlotDayMorning = -1;
int lastSlotDayEvening = -1;

static void sendToUno(const String &line) {
  Serial.println(line);
}

static bool timeValid() {
  time_t now = time(nullptr);
  return now > 1700000000;
}

static int dayOfYear() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) return -1;
  return timeinfo.tm_yday;
}

static bool isScheduleSlotNow(int hour) {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) return false;
  return (timeinfo.tm_hour == hour && timeinfo.tm_min == 0);
}

static void processSchedule() {
  if (inputs.modeSelect != 2) return;
  if (!timeValid()) return;

  int doy = dayOfYear();
  if (doy < 0) return;

  if (isScheduleSlotNow(Config::SLOT_MORNING_H) && lastSlotDayMorning != doy) {
    lastSlotDayMorning = doy;
    sendToUno("CMD:SCHED_07");
  }
  if (isScheduleSlotNow(Config::SLOT_EVENING_H) && lastSlotDayEvening != doy) {
    lastSlotDayEvening = doy;
    sendToUno("CMD:SCHED_17");
  }
}

static void syncToUno() {
  sendToUno(String("SET:MODE=") + inputs.modeSelect);
  sendToUno(String("SET:BIOMASS=") + inputs.biomassG);
  sendToUno(String("SET:PWM=") + inputs.pwmPercent);
  sendToUno(String("SET:GPS100=") + inputs.gramsPerSec100);
  sendToUno(String("SET:SIM_MODE=") + (inputs.simMode ? 1 : 0));
  if (inputs.simMode) {
    sendToUno(String("SET:SIM_TEMP=") + inputs.simTempC);
    sendToUno(String("SET:SIM_DIST=") + inputs.simDistanceCm);
  }
}

static void parseTelemToken(const String &token) {
  int eq = token.indexOf('=');
  if (eq < 0) return;
  String key = token.substring(0, eq);
  String val = token.substring(eq + 1);

  if (key == "T") {
    Blynk.virtualWrite(VPin::TEMP_C, val.toFloat());
  } else if (key == "FEED") {
    Blynk.virtualWrite(VPin::FEED_REMAIN, val.toFloat());
  } else if (key == "BIOMASS") {
    Blynk.virtualWrite(VPin::BIOMASS_OUT, val.toFloat());
  } else if (key == "CMD") {
    Blynk.virtualWrite(VPin::LAST_CMD_GRAMS, val.toInt());
  } else if (key == "PWM") {
    Blynk.virtualWrite(VPin::LAST_PWM, val.toInt());
  } else if (key == "EVENT") {
    Blynk.virtualWrite(VPin::LAST_EVENT, val);
  }
}

static void processSerialLine(const String &line) {
  if (!line.startsWith("TELEM,")) return;
  int start = 6;
  while (start < line.length()) {
    int comma = line.indexOf(',', start);
    if (comma < 0) comma = line.length();
    String token = line.substring(start, comma);
    parseTelemToken(token);
    start = comma + 1;
  }
}

static void pollSerial() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n') {
      if (serialLine.length() > 0) {
        processSerialLine(serialLine);
      }
      serialLine = "";
    } else if (c != '\r') {
      serialLine += c;
      if (serialLine.length() > 200) {
        serialLine = "";
      }
    }
  }
}

BLYNK_CONNECTED() {
  Blynk.syncVirtual(VPin::SIM_TEMP, VPin::BIOMASS, VPin::MANUAL_FEED, VPin::MODE_SELECT,
                    VPin::PWM_PERCENT, VPin::GPS_100, VPin::SIM_EVENT, VPin::TEST_IN);
}

BLYNK_WRITE(VPIN_SIM_TEMP) {
  inputs.simTempC = param.asFloat();
  if (inputs.simMode) {
    sendToUno(String("SET:SIM_TEMP=") + inputs.simTempC);
  }
}

BLYNK_WRITE(VPIN_BIOMASS) {
  inputs.biomassG = param.asFloat();
  sendToUno(String("SET:BIOMASS=") + inputs.biomassG);
}

BLYNK_WRITE(VPIN_MANUAL_FEED) {
  if (param.asInt() == 1) {
    sendToUno("CMD:MANUAL");
    Blynk.virtualWrite(VPin::MANUAL_FEED, 0);
  }
}

BLYNK_WRITE(VPIN_MODE_SELECT) {
  inputs.modeSelect = param.asInt();
  sendToUno(String("SET:MODE=") + inputs.modeSelect);
}

BLYNK_WRITE(VPIN_PWM_PERCENT) {
  inputs.pwmPercent = param.asInt();
  if (inputs.pwmPercent < 0) inputs.pwmPercent = 0;
  if (inputs.pwmPercent > 100) inputs.pwmPercent = 100;
  sendToUno(String("SET:PWM=") + inputs.pwmPercent);
}

BLYNK_WRITE(VPIN_GPS_100) {
  inputs.gramsPerSec100 = param.asInt();
  if (inputs.gramsPerSec100 < 1) inputs.gramsPerSec100 = 1;
  sendToUno(String("SET:GPS100=") + inputs.gramsPerSec100);
}

BLYNK_WRITE(VPIN_SIM_EVENT) {
  int v = param.asInt();
  inputs.simMode = (v == 1);
  sendToUno(String("SET:SIM_MODE=") + (inputs.simMode ? 1 : 0));
  if (inputs.simMode) {
    sendToUno(String("SET:SIM_TEMP=") + inputs.simTempC);
    sendToUno(String("SET:SIM_DIST=") + inputs.simDistanceCm);
    sendToUno("CMD:SIM_EVT");
  }
  Blynk.virtualWrite(VPin::SIM_EVENT, 0);
}

BLYNK_WRITE(VPIN_TEST_IN) {
  inputs.simDistanceCm = param.asFloat();
  if (inputs.simMode) {
    sendToUno(String("SET:SIM_DIST=") + inputs.simDistanceCm);
  }
}

void setup() {
  Serial.begin(Config::UART_BAUD);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  uint32_t startMs = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startMs < Config::WIFI_TIMEOUT_MS) {
    delay(200);
  }

  Blynk.config(BLYNK_AUTH_TOKEN);
  Blynk.connect(Config::BLYNK_CONNECT_TIMEOUT_MS);

  setenv("TZ", "WITA-8", 1);
  tzset();
  configTime(0, 0, "pool.ntp.org", "time.nist.gov", "time.google.com");

  timer.setInterval(1000, processSchedule);
  timer.setInterval(200, pollSerial);
  timer.setInterval(5000, syncToUno);
}

void loop() {
  Blynk.run();
  timer.run();
}
