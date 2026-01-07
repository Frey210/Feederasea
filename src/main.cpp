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
static const uint32_t SAMPLE_INTERVAL_MS = 2000;
static const uint8_t DS18B20_PIN = 4;

static const uint8_t ULTRASONIC_TRIG_PIN = 12;
static const uint8_t ULTRASONIC_ECHO_PIN = 14;

static const uint8_t SERVO_PIN = 26;

static const uint8_t MOTOR_L_PWM = 32;
static const uint8_t MOTOR_R_PWM = 33;
static const uint8_t MOTOR_L_EN = 34;
static const uint8_t MOTOR_R_EN = 35;

static const uint8_t I2C_SDA = 21;
static const uint8_t I2C_SCL = 22;
static const uint8_t LCD_ADDR = 0x27;

static const uint8_t BTN_MANUAL = 39; // input-only
static const uint8_t LED_STATUS = 2;
static const uint8_t LED_LOW_FEED = 15;

static const int PWM_FREQ = 20000;
static const int PWM_RES_BITS = 8;
static const int PWM_CHANNEL_L = 0;
static const int PWM_CHANNEL_R = 1;

static const uint32_t WIFI_TIMEOUT_MS = 15000;
static const uint32_t BLYNK_CONNECT_TIMEOUT_MS = 5000;

// Hopper model (TODO CALIBRATION)
static const float H_TOTAL_CM = 25.0f;
static const float RADIUS_CM = 7.5f;
static const float BULK_DENSITY_G_PER_CM3 = 0.55f;

static const float MASS_MIN_G = 0.0f;
static const float MASS_MAX_G = 50000.0f;

// Debounce
static const uint32_t BTN_DEBOUNCE_MS = 60;

// Servo positions
static const int SERVO_OPEN_DEG = 45;
static const int SERVO_CLOSE_DEG = 0;

// State timings
static const uint32_t SERVO_STABILIZE_MS = 800;
static const uint32_t SERVO_SETTLE_MS = 600;

// Schedule slots
static const int SLOT_MORNING_H = 7;
static const int SLOT_EVENING_H = 17;

// Blynk virtual pins
static const uint8_t VPIN_SIM_TEMP = V1;
static const uint8_t VPIN_BIOMASS = V2;
static const uint8_t VPIN_MANUAL_FEED = V3;
static const uint8_t VPIN_MODE_SELECT = V4;
static const uint8_t VPIN_PWM_PERCENT = V5;
static const uint8_t VPIN_GPS_100 = V6;
static const uint8_t VPIN_SIM_EVENT = V7;
static const uint8_t VPIN_TEST_IN = V8;

static const uint8_t VPIN_TEMP_C = V20;
static const uint8_t VPIN_FEED_REMAIN = V21;
static const uint8_t VPIN_BIOMASS_OUT = V22;
static const uint8_t VPIN_LAST_CMD_GRAMS = V23;
static const uint8_t VPIN_LAST_PWM = V24;
static const uint8_t VPIN_LAST_EVENT = V25;

// Globals
OneWire oneWire(DS18B20_PIN);
DallasTemperature dallas(&oneWire);
LiquidCrystal_I2C lcd(LCD_ADDR, 16, 2);
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

bool useSimulationMode = true;
float simTempC = 28.0f;
double biomassG = 0.0;
int manualFeedSwitch = 0;
int modeSelect = 0;
int pwmPercent = 50;
int gramsPerSec100 = 5;
int simEvent = 0;
uint32_t lastV1UpdateMs = 0;
int lastTestIn = 0;
bool useSimDistance = false;
float simDistanceCm = H_TOTAL_CM;

String lastEventLabel = "NONE";
int lastCmdGrams = 0;
int lastPwm = 0;

String lcdLine1Cache;
String lcdLine2Cache;

uint32_t lastSampleMs = 0;
uint32_t lastBtnChangeMs = 0;
int lastBtnState = HIGH;

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
  ledcSetup(PWM_CHANNEL_L, PWM_FREQ, PWM_RES_BITS);
  ledcSetup(PWM_CHANNEL_R, PWM_FREQ, PWM_RES_BITS);
  ledcAttachPin(MOTOR_L_PWM, PWM_CHANNEL_L);
  ledcAttachPin(MOTOR_R_PWM, PWM_CHANNEL_R);
  ledcWrite(PWM_CHANNEL_L, 0);
  ledcWrite(PWM_CHANNEL_R, 0);
}

static void motorStop() {
  ledcWrite(PWM_CHANNEL_L, 0);
  ledcWrite(PWM_CHANNEL_R, 0);
  digitalWrite(MOTOR_L_EN, LOW);
  digitalWrite(MOTOR_R_EN, LOW);
}

static void motorForward(int pwm) {
  int duty = map(pwm, 0, 100, 0, 255);
  digitalWrite(MOTOR_L_EN, HIGH);
  digitalWrite(MOTOR_R_EN, HIGH);
  ledcWrite(PWM_CHANNEL_L, 0);
  ledcWrite(PWM_CHANNEL_R, duty);
}

static float readUltrasonicCm() {
  // Take 5 samples and average with basic outlier rejection
  const int samples = 5;
  float sum = 0.0f;
  int valid = 0;

  for (int i = 0; i < samples; i++) {
    digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(ULTRASONIC_TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(ULTRASONIC_TRIG_PIN, LOW);

    unsigned long duration = pulseIn(ULTRASONIC_ECHO_PIN, HIGH, 30000);
    float distanceCm;

    if (duration == 0) {
      distanceCm = H_TOTAL_CM;
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
    return H_TOTAL_CM;
  }
  return sum / valid;
}

static float estimateFeedMassG(float distanceCm) {
  float heightFilled = clampf(H_TOTAL_CM - distanceCm, 0.0f, H_TOTAL_CM);
  float volume = PI * RADIUS_CM * RADIUS_CM * heightFilled;
  float mass = volume * BULK_DENSITY_G_PER_CM3;
  return clampf(mass, MASS_MIN_G, MASS_MAX_G);
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
  if (modeSelect == 0) {
    return 50;
  }
  float pct = (tempC >= 25.0f && tempC <= 37.0f) ? 0.03f : 0.02f;
  return (int)round(biomassG * pct);
}

static void startFeedEvent(const String &eventLabel) {
  lastEventLabel = eventLabel;
  lastCmdGrams = computeCommandedGrams(lastTempC);
  lastPwm = pwmPercent;

  double gps100 = (gramsPerSec100 < 1) ? 1.0 : gramsPerSec100;
  double gramsPerSec = gps100 * (pwmPercent / 100.0);
  if (gramsPerSec < 0.1) gramsPerSec = 0.1;

  double runtimeS = lastCmdGrams / gramsPerSec;
  if (runtimeS > 12.0) runtimeS = 12.0;

  motorRunMs = (uint32_t)(runtimeS * 1000.0);

  feedState = PRECHECK;
  stateStartMs = millis();
}

static void updateBlynkTelemetry() {
  Blynk.virtualWrite(VPIN_TEMP_C, lastTempCReal);
  Blynk.virtualWrite(VPIN_FEED_REMAIN, lastFeedRemainingG);
  Blynk.virtualWrite(VPIN_BIOMASS_OUT, biomassG);
  Blynk.virtualWrite(VPIN_LAST_CMD_GRAMS, lastCmdGrams);
  Blynk.virtualWrite(VPIN_LAST_PWM, lastPwm);
  Blynk.virtualWrite(VPIN_LAST_EVENT, lastEventLabel);
}

static void samplingTask() {
  dallas.requestTemperatures();
  float tempC = dallas.getTempCByIndex(0);
  if (tempC > -100 && tempC < 150) {
    lastTempCReal = tempC;
  }

  float distanceRawCm = readUltrasonicCm();
  float distanceCm = useSimDistance ? simDistanceCm : distanceRawCm;
  distanceCm = clampf(distanceCm, 0.0f, H_TOTAL_CM);
  lastFeedRemainingG = estimateFeedMassG(distanceCm);

  float effectiveTemp = useSimulationMode ? simTempC : lastTempCReal;
  if (!isnan(effectiveTemp)) {
    lastTempC = effectiveTemp;
  }

  String realStr = isnan(lastTempCReal) ? "--" : String(lastTempCReal, 1);
  String line1 = "M:" + modeToChar(modeSelect) + " R:" + realStr;
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
      (unsigned long)((millis() - lastV1UpdateMs) / 1000UL),
      lastTestIn,
      lastEventLabel.c_str());
  Serial.println(logbuf);
}

static void syncInputs() {
  if (Blynk.connected()) {
    Blynk.syncVirtual(VPIN_SIM_TEMP, VPIN_BIOMASS, VPIN_MANUAL_FEED, VPIN_MODE_SELECT,
                      VPIN_PWM_PERCENT, VPIN_GPS_100, VPIN_SIM_EVENT, VPIN_TEST_IN);
  }
}

static void handleButton() {
  int raw = digitalRead(BTN_MANUAL);
  uint32_t now = millis();

  if (raw != lastBtnState) {
    lastBtnChangeMs = now;
    lastBtnState = raw;
  }

  if (now - lastBtnChangeMs > BTN_DEBOUNCE_MS) {
    if (raw == LOW && manualFeedSwitch == 0) {
      manualFeedSwitch = 1;
      startFeedEvent(makeEventLabel("MANUAL"));
      Blynk.virtualWrite(VPIN_MANUAL_FEED, 1);
    }
  }
}

static void processSchedule() {
  if (modeSelect != 2) return;
  if (!timeValid()) return;

  int doy = dayOfYear();
  if (doy < 0) return;

  if (isScheduleSlotNow(SLOT_MORNING_H) && lastSlotDayMorning != doy) {
    lastSlotDayMorning = doy;
    startFeedEvent(makeEventLabel("SCHED_07"));
  }
  if (isScheduleSlotNow(SLOT_EVENING_H) && lastSlotDayEvening != doy) {
    lastSlotDayEvening = doy;
    startFeedEvent(makeEventLabel("SCHED_17"));
  }
}

static void updateFeedState() {
  uint32_t now = millis();

  switch (feedState) {
    case IDLE:
      digitalWrite(LED_LOW_FEED, LOW);
      break;

    case PRECHECK:
      if (modeSelect == 2) {
        if (lastFeedRemainingG + 1.0f < lastCmdGrams) {
          feedState = ABORT_LOW_FEED;
          stateStartMs = now;
          digitalWrite(LED_LOW_FEED, HIGH);
          Blynk.logEvent("low_feed");
          lastEventLabel = lastEventLabel + "_ABORT";
          break;
        }
      }
      valveServo.write(SERVO_OPEN_DEG);
      feedState = SERVO_OPENING;
      stateStartMs = now;
      break;

    case SERVO_OPENING:
      if (now - stateStartMs >= SERVO_STABILIZE_MS) {
        motorForward(pwmPercent);
        feedState = MOTOR_RUNNING;
        stateStartMs = now;
      }
      break;

    case MOTOR_RUNNING:
      if (now - stateStartMs >= motorRunMs) {
        motorStop();
        valveServo.write(SERVO_CLOSE_DEG);
        feedState = SERVO_CLOSING;
        stateStartMs = now;
      }
      break;

    case SERVO_CLOSING:   
      if (now - stateStartMs >= SERVO_SETTLE_MS) {
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
  Blynk.syncVirtual(VPIN_SIM_TEMP, VPIN_BIOMASS, VPIN_MANUAL_FEED, VPIN_MODE_SELECT,
                    VPIN_PWM_PERCENT, VPIN_GPS_100, VPIN_SIM_EVENT, VPIN_TEST_IN);
  Serial.println("Blynk connected, sync virtual pins");
  Blynk.virtualWrite(VPIN_LAST_EVENT, "BLYNK_CONNECTED");
}

BLYNK_WRITE_DEFAULT() {
  int pin = request.pin;
  String val = param.asStr();
  Serial.printf("Blynk V%d raw=%s\n", pin, val.c_str());
}

BLYNK_WRITE(VPIN_SIM_TEMP) {
  simTempC = param.asDouble();
  useSimulationMode = true;
  lastTempC = simTempC;
  lastV1UpdateMs = millis();
  Serial.printf("Blynk V1 Sim_Temp=%.2fC (simulation ON)\n", simTempC);
}

BLYNK_WRITE(VPIN_BIOMASS) {
  biomassG = param.asDouble();
  Serial.printf("Blynk V2 Biomass=%.2fg\n", biomassG);
}

BLYNK_WRITE(VPIN_MANUAL_FEED) {
  manualFeedSwitch = param.asInt();
  Serial.printf("Blynk V3 Manual_Feed=%d\n", manualFeedSwitch);
  if (manualFeedSwitch == 1) {
    startFeedEvent(makeEventLabel("MANUAL"));
    Blynk.virtualWrite(VPIN_MANUAL_FEED, 0);
  }
}

BLYNK_WRITE(VPIN_MODE_SELECT) {
  modeSelect = param.asInt();
  Serial.printf("Blynk V4 Mode_Select=%d\n", modeSelect);
}

BLYNK_WRITE(VPIN_PWM_PERCENT) {
  pwmPercent = param.asInt();
  if (pwmPercent < 0) pwmPercent = 0;
  if (pwmPercent > 100) pwmPercent = 100;
  Serial.printf("Blynk V5 PWM_Percent=%d\n", pwmPercent);
}

BLYNK_WRITE(VPIN_GPS_100) {
  gramsPerSec100 = param.asInt();
  if (gramsPerSec100 < 1) gramsPerSec100 = 1;
  Serial.printf("Blynk V6 GramPerSec_100=%d\n", gramsPerSec100);
}

BLYNK_WRITE(VPIN_SIM_EVENT) {
  simEvent = param.asInt();
  Serial.printf("Blynk V7 Sim_Event=%d\n", simEvent);
  if (simEvent == 1) {
    startFeedEvent(makeEventLabel("SIM_EVT"));
    Blynk.virtualWrite(VPIN_SIM_EVENT, 0);
  }
}

BLYNK_WRITE(VPIN_TEST_IN) {
  float v = param.asFloat();
  lastTestIn = (int)round(v);
  simDistanceCm = v;
  useSimDistance = true;
  Serial.printf("Blynk V8 Test_In=%.2fcm (sim distance ON)\n", v);
  Blynk.virtualWrite(VPIN_LAST_EVENT, String("V8=") + v);
}

void setup() {
  Serial.begin(115200);
  Serial.setTxBufferSize(1024);

  pinMode(ULTRASONIC_TRIG_PIN, OUTPUT);
  pinMode(ULTRASONIC_ECHO_PIN, INPUT);

  pinMode(MOTOR_L_PWM, OUTPUT);
  pinMode(MOTOR_R_PWM, OUTPUT);
  pinMode(MOTOR_L_EN, OUTPUT);
  pinMode(MOTOR_R_EN, OUTPUT);
  setupPwm();
  motorStop();

  pinMode(LED_STATUS, OUTPUT);
  pinMode(LED_LOW_FEED, OUTPUT);
  digitalWrite(LED_STATUS, LOW);
  digitalWrite(LED_LOW_FEED, LOW);

  pinMode(BTN_MANUAL, INPUT);

  valveServo.attach(SERVO_PIN);
  valveServo.write(SERVO_CLOSE_DEG);

  Wire.begin(I2C_SDA, I2C_SCL);
  lcd.init();
  lcd.backlight();
  lcdSetLines("Booting...", "Please wait");

  dallas.begin();

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  uint32_t startMs = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startMs < WIFI_TIMEOUT_MS) {
    lcdSetLines("WiFi", "Connecting...");
    delay(200);
  }

  if (WiFi.status() == WL_CONNECTED) {
    lcdSetLines("WiFi", "OK");
  } else {
    lcdSetLines("WiFi", "FAIL");
  }

  Blynk.config(BLYNK_AUTH_TOKEN);
  Blynk.connect(BLYNK_CONNECT_TIMEOUT_MS);

  setenv("TZ", "WITA-8", 1);
  tzset();
  configTime(0, 0, "pool.ntp.org", "time.nist.gov", "time.google.com");

  timer.setInterval(SAMPLE_INTERVAL_MS, samplingTask);
  timer.setInterval(10000, syncInputs);
}

void loop() {
  Blynk.run();
  timer.run();

  handleButton();
  processSchedule();
  updateFeedState();

  digitalWrite(LED_STATUS, (WiFi.status() == WL_CONNECTED && Blynk.connected()) ? HIGH : LOW);
}
