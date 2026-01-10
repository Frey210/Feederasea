// src/main.cpp
#include <Arduino.h>
#include "secrets.h"
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <LiquidCrystal_I2C.h>
#include <ESP32Servo.h>
#include <time.h>

// =========================
// CONFIG
// =========================
namespace Pins {
static const uint8_t DS18B20 = 4;
static const uint8_t ULTRASONIC_TRIG = 12;
static const uint8_t ULTRASONIC_ECHO = 14;
static const uint8_t SERVO = 26;
static const uint8_t MOTOR_L_PWM = 32;
static const uint8_t MOTOR_R_PWM = 33;
static const uint8_t MOTOR_L_EN = 27; // output-capable (GPIO34/35 are input-only)
static const uint8_t MOTOR_R_EN = 25;
static const uint8_t I2C_SDA = 21;
static const uint8_t I2C_SCL = 22;
static const uint8_t LCD_ADDR = 0x27;
static const uint8_t BTN_MANUAL = 34; // input-only, needs external pull-up
static const uint8_t LED_STATUS = 2;
static const uint8_t LED_LOW_FEED = 15;
}  // namespace Pins

namespace Config {
static const uint32_t SAMPLE_INTERVAL_MS = 2000;
static const int PWM_FREQ = 20000;
static const int PWM_RES_BITS = 8;
static const int PWM_CHANNEL_L = 0;
static const int PWM_CHANNEL_R = 1;
static const uint32_t WIFI_TIMEOUT_MS = 15000;
static const uint32_t BLYNK_CONNECT_TIMEOUT_MS = 5000;
static const float H_TOTAL_CM = 25.0f;
static const float RADIUS_CM = 7.5f;
static const float BULK_DENSITY_G_PER_CM3 = 0.55f;
static const float MASS_MIN_G = 0.0f;
static const float MASS_MAX_G = 50000.0f;
static const uint32_t BTN_DEBOUNCE_MS = 60;
static const int SERVO_OPEN_DEG = 45;
static const int SERVO_CLOSE_DEG = 0;
static const uint32_t SERVO_STABILIZE_MS = 800;
static const uint32_t SERVO_SETTLE_MS = 600;
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

// Macros for BLYNK_WRITE token pasting
#define VPIN_SIM_TEMP V1
#define VPIN_BIOMASS V2
#define VPIN_MANUAL_FEED V3
#define VPIN_MODE_SELECT V4
#define VPIN_PWM_PERCENT V5
#define VPIN_GPS_100 V6
#define VPIN_SIM_EVENT V7
#define VPIN_TEST_IN V8

// Globals
OneWire oneWire(Pins::DS18B20);
DallasTemperature dallas(&oneWire);
LiquidCrystal_I2C lcd(Pins::LCD_ADDR, 16, 2);
Servo valveServo;

BlynkTimer timer;

enum FeedState {
  IDLE,
  PRECHECK,
  SERVO_OPENING,
  MOTOR_RUNNING,
  SERVO_CLOSING,
  POSTLOG,
  ABORT_LOW_FEED
};

FeedState feedState = IDLE;
uint32_t stateStartMs = 0;
uint32_t motorRunMs = 0;
float lastTempC = NAN;      // effective temp used by logic/UI
float lastTempCReal = NAN;  // sensor temp for diagnostics
float lastFeedRemainingG = 0.0f;

struct AppInputs {
  bool simMode = false;
  float simTempC = 28.0f;
  double biomassG = 0.0;
  int modeSelect = 0;
  int pwmPercent = 50;
  int gramsPerSec100 = 5;
  float simDistanceCm = Config::H_TOTAL_CM;
  uint32_t lastSimTempUpdateMs = 0;
};

struct AppRequests {
  bool manualFeed = false;
  bool simEvent = false;
};

static AppInputs inputs;
static AppRequests requests;

String lastEventLabel = "NONE";
int lastCmdGrams = 0;
int lastPwm = 0;

String lcdLine1Cache;
String lcdLine2Cache;

uint32_t lastBtnChangeMs = 0;
int lastBtnState = HIGH;
bool btnLatched = false;

int lastSlotDayMorning = -1;
int lastSlotDayEvening = -1;

static float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static void lcdSetLines(const String &line1, const String &line2) {
  if (line1 != lcdLine1Cache) {
    lcd.setCursor(0, 0);
    lcd.print("                ");
    lcd.setCursor(0, 0);
    lcd.print(line1);
    lcdLine1Cache = line1;
  }
  if (line2 != lcdLine2Cache) {
    lcd.setCursor(0, 1);
    lcd.print("                ");
    lcd.setCursor(0, 1);
    lcd.print(line2);
    lcdLine2Cache = line2;
  }
}

static void setupPwm() {
  ledcSetup(Config::PWM_CHANNEL_L, Config::PWM_FREQ, Config::PWM_RES_BITS);
  ledcSetup(Config::PWM_CHANNEL_R, Config::PWM_FREQ, Config::PWM_RES_BITS);
  ledcAttachPin(Pins::MOTOR_L_PWM, Config::PWM_CHANNEL_L);
  ledcAttachPin(Pins::MOTOR_R_PWM, Config::PWM_CHANNEL_R);
  ledcWrite(Config::PWM_CHANNEL_L, 0);
  ledcWrite(Config::PWM_CHANNEL_R, 0);
}

static void motorStop() {
  ledcWrite(Config::PWM_CHANNEL_L, 0);
  ledcWrite(Config::PWM_CHANNEL_R, 0);
  digitalWrite(Pins::MOTOR_L_EN, LOW);
  digitalWrite(Pins::MOTOR_R_EN, LOW);
}

static void motorForward(int pwm) {
  int duty = map(pwm, 0, 100, 0, 255);
  digitalWrite(Pins::MOTOR_L_EN, HIGH);
  digitalWrite(Pins::MOTOR_R_EN, HIGH);
  ledcWrite(Config::PWM_CHANNEL_L, 0);
  ledcWrite(Config::PWM_CHANNEL_R, duty);
}

static float readUltrasonicCm() {
  // Take 5 samples and average with basic outlier rejection
  const int samples = 5;
  float sum = 0.0f;
  int valid = 0;

  for (int i = 0; i < samples; i++) {
    digitalWrite(Pins::ULTRASONIC_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(Pins::ULTRASONIC_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(Pins::ULTRASONIC_TRIG, LOW);

    unsigned long duration = pulseIn(Pins::ULTRASONIC_ECHO, HIGH, 30000);
    float distanceCm;

    if (duration == 0) {
      distanceCm = Config::H_TOTAL_CM;
    } else {
      distanceCm = (duration * 0.0343f) / 2.0f;
    }

    if (distanceCm >= 1.0f && distanceCm <= 400.0f) {
      sum += distanceCm;
      valid++;
    }
    delay(10);
  }

  if (valid == 0) {
    return Config::H_TOTAL_CM;
  }
  return sum / valid;
}

static float estimateFeedMassG(float distanceCm) {
  float heightFilled = clampf(Config::H_TOTAL_CM - distanceCm, 0.0f, Config::H_TOTAL_CM);
  float volume = PI * Config::RADIUS_CM * Config::RADIUS_CM * heightFilled;
  float mass = volume * Config::BULK_DENSITY_G_PER_CM3;
  return clampf(mass, Config::MASS_MIN_G, Config::MASS_MAX_G);
}

static String modeToChar(int mode) {
  if (mode == 0) return "A";
  if (mode == 1) return "B";
  return "C";
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

static String makeEventLabel(const String &base) {
  return base;
}

static int computeCommandedGrams(float tempC) {
  if (inputs.modeSelect == 0) {
    return 50;
  }
  float pct = (tempC >= 25.0f && tempC <= 37.0f) ? 0.03f : 0.02f;
  return (int)round(inputs.biomassG * pct);
}

static void startFeedEvent(const String &eventLabel) {
  lastEventLabel = eventLabel;
  lastCmdGrams = computeCommandedGrams(lastTempC);
  lastPwm = inputs.pwmPercent;

  double gps100 = (inputs.gramsPerSec100 < 1) ? 1.0 : inputs.gramsPerSec100;
  double gramsPerSec = gps100 * (inputs.pwmPercent / 100.0);
  if (gramsPerSec < 0.1) gramsPerSec = 0.1;

  double runtimeS = lastCmdGrams / gramsPerSec;
  if (runtimeS > 12.0) runtimeS = 12.0;

  motorRunMs = (uint32_t)(runtimeS * 1000.0);

  feedState = PRECHECK;
  stateStartMs = millis();
}

static void updateBlynkTelemetry() {
  Blynk.virtualWrite(VPin::TEMP_C, lastTempCReal);
  Blynk.virtualWrite(VPin::FEED_REMAIN, lastFeedRemainingG);
  Blynk.virtualWrite(VPin::BIOMASS_OUT, inputs.biomassG);
  Blynk.virtualWrite(VPin::LAST_CMD_GRAMS, lastCmdGrams);
  Blynk.virtualWrite(VPin::LAST_PWM, lastPwm);
  Blynk.virtualWrite(VPin::LAST_EVENT, lastEventLabel);
}

static void samplingTask() {
  dallas.requestTemperatures();
  float tempC = dallas.getTempCByIndex(0);
  if (tempC > -100 && tempC < 150) {
    lastTempCReal = tempC;
  }

  float distanceRawCm = readUltrasonicCm();
  float distanceCm = inputs.simMode ? inputs.simDistanceCm : distanceRawCm;
  distanceCm = clampf(distanceCm, 0.0f, Config::H_TOTAL_CM);
  lastFeedRemainingG = estimateFeedMassG(distanceCm);

  float effectiveTemp = inputs.simMode ? inputs.simTempC : lastTempCReal;
  if (!isnan(effectiveTemp)) {
    lastTempC = effectiveTemp;
  }

  String realStr = isnan(lastTempCReal) ? "--" : String(lastTempCReal, 1);
  String line1 = "M:" + modeToChar(inputs.modeSelect) + " R:" + realStr;
  String line2;
  if (feedState == IDLE) {
    line2 = "D:" + String(distanceCm, 1) + " F:" + String((int)lastFeedRemainingG);
  } else if (feedState == ABORT_LOW_FEED) {
    line2 = "LOW FEED!";
  } else {
    line2 = "Feeding...";
  }

  lcdSetLines(line1, line2);
  updateBlynkTelemetry();

  char logbuf[200];
  snprintf(
      logbuf,
      sizeof(logbuf),
      "T=%.2fC (real=%.2fC) Dist=%.1fcm (raw=%.1fcm) Feed=%.0fg State=%d WiFi=%s Blynk=%s V1_age=%lus V8=%d Event=%s",
      lastTempC,
      lastTempCReal,
      distanceCm,
      distanceRawCm,
      lastFeedRemainingG,
      (int)feedState,
      (WiFi.status() == WL_CONNECTED) ? "OK" : "FAIL",
      Blynk.connected() ? "OK" : "FAIL",
      (unsigned long)((millis() - inputs.lastSimTempUpdateMs) / 1000UL),
      inputs.simMode ? (int)round(inputs.simDistanceCm) : -1,
      lastEventLabel.c_str());
  Serial.println(logbuf);
}

static void handleButton() {
  int raw = digitalRead(Pins::BTN_MANUAL);
  uint32_t now = millis();

  if (raw != lastBtnState) {
    lastBtnChangeMs = now;
    lastBtnState = raw;
  }

  if (now - lastBtnChangeMs > Config::BTN_DEBOUNCE_MS) {
    if (raw == LOW && !btnLatched && !requests.manualFeed) {
      requests.manualFeed = true;
      btnLatched = true;
    }
    if (raw == HIGH) {
      btnLatched = false;
    }
  }
}

static void processSchedule() {
  if (inputs.modeSelect != 2) return;
  if (!timeValid()) return;

  int doy = dayOfYear();
  if (doy < 0) return;

  if (isScheduleSlotNow(Config::SLOT_MORNING_H) && lastSlotDayMorning != doy) {
    lastSlotDayMorning = doy;
    startFeedEvent(makeEventLabel("SCHED_07"));
  }
  if (isScheduleSlotNow(Config::SLOT_EVENING_H) && lastSlotDayEvening != doy) {
    lastSlotDayEvening = doy;
    startFeedEvent(makeEventLabel("SCHED_17"));
  }
}

static void updateFeedState() {
  uint32_t now = millis();

  switch (feedState) {
    case IDLE:
      digitalWrite(Pins::LED_LOW_FEED, LOW);
      break;

    case PRECHECK:
      if (inputs.modeSelect == 2) {
        if (lastFeedRemainingG + 1.0f < lastCmdGrams) {
          feedState = ABORT_LOW_FEED;
          stateStartMs = now;
          digitalWrite(Pins::LED_LOW_FEED, HIGH);
          Blynk.logEvent("low_feed");
          lastEventLabel = lastEventLabel + "_ABORT";
          break;
        }
      }
      valveServo.write(Config::SERVO_OPEN_DEG);
      feedState = SERVO_OPENING;
      stateStartMs = now;
      break;

    case SERVO_OPENING:
      if (now - stateStartMs >= Config::SERVO_STABILIZE_MS) {
        motorForward(inputs.pwmPercent);
        feedState = MOTOR_RUNNING;
        stateStartMs = now;
      }
      break;

    case MOTOR_RUNNING:
      if (now - stateStartMs >= motorRunMs) {
        motorStop();
        valveServo.write(Config::SERVO_CLOSE_DEG);
        feedState = SERVO_CLOSING;
        stateStartMs = now;
      }
      break;

    case SERVO_CLOSING:   
      if (now - stateStartMs >= Config::SERVO_SETTLE_MS) {
        feedState = POSTLOG;
        stateStartMs = now;
      }
      break;

    case POSTLOG:
      Blynk.logEvent("feed_done");
      Blynk.logEvent("dispense_done");
      feedState = IDLE;
      stateStartMs = now;
      break;

    case ABORT_LOW_FEED:
      if (now - stateStartMs >= 3000) {
        feedState = IDLE;
        stateStartMs = now;
      }
      break;
  }
}

BLYNK_CONNECTED() {
  Blynk.syncVirtual(VPin::SIM_TEMP, VPin::BIOMASS, VPin::MANUAL_FEED, VPin::MODE_SELECT,
                    VPin::PWM_PERCENT, VPin::GPS_100, VPin::SIM_EVENT, VPin::TEST_IN);
  Serial.println("Blynk connected, sync virtual pins");
  Blynk.virtualWrite(VPin::LAST_EVENT, "BLYNK_CONNECTED");
}

BLYNK_WRITE_DEFAULT() {
  int pin = request.pin;
  String val = param.asStr();
  Serial.printf("Blynk V%d raw=%s\n", pin, val.c_str());
}

BLYNK_WRITE(VPIN_SIM_TEMP) {
  inputs.simTempC = param.asDouble();
  inputs.lastSimTempUpdateMs = millis();
  if (inputs.simMode) {
    lastTempC = inputs.simTempC;
    Serial.printf("Blynk V1 Sim_Temp=%.2fC (simulation ON)\n", inputs.simTempC);
  } else {
    Serial.printf("Blynk V1 Sim_Temp=%.2fC (ignored, sim OFF)\n", inputs.simTempC);
  }
}

BLYNK_WRITE(VPIN_BIOMASS) {
  inputs.biomassG = param.asDouble();
  Serial.printf("Blynk V2 Biomass=%.2fg\n", inputs.biomassG);
}

BLYNK_WRITE(VPIN_MANUAL_FEED) {
  if (param.asInt() == 1) {
    requests.manualFeed = true;
  }
  Serial.printf("Blynk V3 Manual_Feed=%d\n", param.asInt());
}

BLYNK_WRITE(VPIN_MODE_SELECT) {
  inputs.modeSelect = param.asInt();
  Serial.printf("Blynk V4 Mode_Select=%d\n", inputs.modeSelect);
}

BLYNK_WRITE(VPIN_PWM_PERCENT) {
  inputs.pwmPercent = param.asInt();
  if (inputs.pwmPercent < 0) inputs.pwmPercent = 0;
  if (inputs.pwmPercent > 100) inputs.pwmPercent = 100;
  Serial.printf("Blynk V5 PWM_Percent=%d\n", inputs.pwmPercent);
}

BLYNK_WRITE(VPIN_GPS_100) {
  inputs.gramsPerSec100 = param.asInt();
  if (inputs.gramsPerSec100 < 1) inputs.gramsPerSec100 = 1;
  Serial.printf("Blynk V6 GramPerSec_100=%d\n", inputs.gramsPerSec100);
}

BLYNK_WRITE(VPIN_SIM_EVENT) {
  int v = param.asInt();
  inputs.simMode = (v == 1);
  if (inputs.simMode) {
    requests.simEvent = true;
  }
  Serial.printf("Blynk V7 Sim_Event=%d (simMode=%d)\n", v, inputs.simMode ? 1 : 0);
}

BLYNK_WRITE(VPIN_TEST_IN) {
  float v = param.asFloat();
  inputs.simDistanceCm = v;
  if (inputs.simMode) {
    Serial.printf("Blynk V8 Test_In=%.2fcm (sim distance ON)\n", v);
  } else {
    Serial.printf("Blynk V8 Test_In=%.2fcm (ignored, sim OFF)\n", v);
  }
  Blynk.virtualWrite(VPin::LAST_EVENT, String("V8=") + v);
}

void setup() {
  Serial.begin(115200);
  Serial.setTxBufferSize(1024);

  pinMode(Pins::ULTRASONIC_TRIG, OUTPUT);
  pinMode(Pins::ULTRASONIC_ECHO, INPUT);

  pinMode(Pins::MOTOR_L_PWM, OUTPUT);
  pinMode(Pins::MOTOR_R_PWM, OUTPUT);
  pinMode(Pins::MOTOR_L_EN, OUTPUT);
  pinMode(Pins::MOTOR_R_EN, OUTPUT);
  setupPwm();
  motorStop();

  pinMode(Pins::LED_STATUS, OUTPUT);
  pinMode(Pins::LED_LOW_FEED, OUTPUT);
  digitalWrite(Pins::LED_STATUS, LOW);
  digitalWrite(Pins::LED_LOW_FEED, LOW);

  pinMode(Pins::BTN_MANUAL, INPUT);

  valveServo.attach(Pins::SERVO);
  valveServo.write(Config::SERVO_CLOSE_DEG);

  Wire.begin(Pins::I2C_SDA, Pins::I2C_SCL);
  lcd.init();
  lcd.backlight();
  lcdSetLines("Booting...", "Please wait");

  dallas.begin();

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  uint32_t startMs = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startMs < Config::WIFI_TIMEOUT_MS) {
    lcdSetLines("WiFi", "Connecting...");
    delay(200);
  }

  if (WiFi.status() == WL_CONNECTED) {
    lcdSetLines("WiFi", "OK");
  } else {
    lcdSetLines("WiFi", "FAIL");
  }

  Blynk.config(BLYNK_AUTH_TOKEN);
  Blynk.connect(Config::BLYNK_CONNECT_TIMEOUT_MS);

  setenv("TZ", "WITA-8", 1);
  tzset();
  configTime(0, 0, "pool.ntp.org", "time.nist.gov", "time.google.com");

  timer.setInterval(Config::SAMPLE_INTERVAL_MS, samplingTask);
}

void loop() {
  Blynk.run();
  timer.run();

  handleButton();
  if (requests.manualFeed) {
    requests.manualFeed = false;
    if (feedState == IDLE) {
      startFeedEvent(makeEventLabel("MANUAL"));
      Blynk.virtualWrite(VPin::MANUAL_FEED, 0);
    }
  }
  if (requests.simEvent) {
    requests.simEvent = false;
    if (feedState == IDLE) {
      startFeedEvent(makeEventLabel("SIM_EVT"));
      Blynk.virtualWrite(VPin::SIM_EVENT, 0);
    }
  }
  processSchedule();
  updateFeedState();

  digitalWrite(Pins::LED_STATUS, (WiFi.status() == WL_CONNECTED && Blynk.connected()) ? HIGH : LOW);
}
