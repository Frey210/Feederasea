// firmware/uno/feeder_uno.ino
#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <LiquidCrystal_I2C.h>
#include <Servo.h>
#include <Wire.h>

namespace Pins {
static const uint8_t DS18B20 = 4;
static const uint8_t ULTRASONIC_TRIG = 12;
static const uint8_t ULTRASONIC_ECHO = 11;
static const uint8_t SERVO = 9;
static const uint8_t MOTOR_L_PWM = 5;
static const uint8_t MOTOR_R_PWM = 6;
static const uint8_t MOTOR_L_EN = 7;
static const uint8_t MOTOR_R_EN = 8;
static const uint8_t LCD_ADDR = 0x27;
static const uint8_t BTN_MANUAL = 2;
static const uint8_t LED_STATUS = 13;
static const uint8_t LED_LOW_FEED = 10;
}  // namespace Pins

namespace Config {
static const uint32_t SAMPLE_INTERVAL_MS = 2000;
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
static const uint32_t SERIAL_BAUD = 9600;
}  // namespace Config

OneWire oneWire(Pins::DS18B20);
DallasTemperature dallas(&oneWire);
LiquidCrystal_I2C lcd(Pins::LCD_ADDR, 16, 2);
Servo valveServo;

enum FeedState {
  IDLE,
  PRECHECK,
  SERVO_OPENING,
  MOTOR_RUNNING,
  SERVO_CLOSING,
  POSTLOG,
  ABORT_LOW_FEED
};

struct AppInputs {
  bool simMode = false;
  float simTempC = 28.0f;
  float simDistanceCm = Config::H_TOTAL_CM;
  double biomassG = 0.0;
  int modeSelect = 0;
  int pwmPercent = 50;
  int gramsPerSec100 = 5;
};

struct AppRequests {
  bool manualFeed = false;
  bool simEvent = false;
  bool schedMorning = false;
  bool schedEvening = false;
};

static AppInputs inputs;
static AppRequests requests;

FeedState feedState = IDLE;
uint32_t stateStartMs = 0;
uint32_t motorRunMs = 0;
float lastTempC = NAN;
float lastTempCReal = NAN;
float lastFeedRemainingG = 0.0f;
float lastDistanceCm = Config::H_TOTAL_CM;

int lastCmdGrams = 0;
int lastPwm = 0;
char lastEventLabel[20] = "NONE";

String lcdLine1Cache;
String lcdLine2Cache;

uint32_t lastSampleMs = 0;
uint32_t lastBtnChangeMs = 0;
int lastBtnState = HIGH;
bool btnLatched = false;

char serialBuf[120];
uint8_t serialPos = 0;

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

static void motorStop() {
  analogWrite(Pins::MOTOR_L_PWM, 0);
  analogWrite(Pins::MOTOR_R_PWM, 0);
  digitalWrite(Pins::MOTOR_L_EN, LOW);
  digitalWrite(Pins::MOTOR_R_EN, LOW);
}

static void motorForward(int pwm) {
  int duty = map(pwm, 0, 100, 0, 255);
  digitalWrite(Pins::MOTOR_L_EN, HIGH);
  digitalWrite(Pins::MOTOR_R_EN, HIGH);
  analogWrite(Pins::MOTOR_L_PWM, 0);
  analogWrite(Pins::MOTOR_R_PWM, duty);
}

static float readUltrasonicCm() {
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

static const char *modeToChar(int mode) {
  if (mode == 0) return "A";
  if (mode == 1) return "B";
  return "C";
}

static int computeCommandedGrams(float tempC) {
  if (inputs.modeSelect == 0) {
    return 50;
  }
  float pct = (tempC >= 25.0f && tempC <= 37.0f) ? 0.03f : 0.02f;
  return (int)round(inputs.biomassG * pct);
}

static void startFeedEvent(const char *eventLabel) {
  strncpy(lastEventLabel, eventLabel, sizeof(lastEventLabel) - 1);
  lastEventLabel[sizeof(lastEventLabel) - 1] = '\0';
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

static void sendTelemetry() {
  char buf[220];
  snprintf(
      buf,
      sizeof(buf),
      "TELEM,T=%.2f,TR=%.2f,DIST=%.1f,FEED=%.0f,MODE=%d,STATE=%d,EVENT=%s,BIOMASS=%.0f,CMD=%d,PWM=%d,SIM=%d",
      lastTempC,
      lastTempCReal,
      lastDistanceCm,
      lastFeedRemainingG,
      inputs.modeSelect,
      (int)feedState,
      lastEventLabel,
      inputs.biomassG,
      lastCmdGrams,
      lastPwm,
      inputs.simMode ? 1 : 0);
  Serial.println(buf);
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
  lastDistanceCm = distanceCm;
  lastFeedRemainingG = estimateFeedMassG(distanceCm);

  float effectiveTemp = inputs.simMode ? inputs.simTempC : lastTempCReal;
  if (!isnan(effectiveTemp)) {
    lastTempC = effectiveTemp;
  }

  String realStr = isnan(lastTempCReal) ? "--" : String(lastTempCReal, 1);
  String line1 = "M:" + String(modeToChar(inputs.modeSelect)) + " R:" + realStr;
  String line2;
  if (feedState == IDLE) {
    line2 = "D:" + String(distanceCm, 1) + " F:" + String((int)lastFeedRemainingG);
  } else if (feedState == ABORT_LOW_FEED) {
    line2 = "LOW FEED!";
  } else {
    line2 = "Feeding...";
  }

  lcdSetLines(line1, line2);
  sendTelemetry();
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
          strncat(lastEventLabel, "_ABORT", sizeof(lastEventLabel) - strlen(lastEventLabel) - 1);
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

static void handleSerialCommand(const char *line) {
  if (strncmp(line, "SET:", 4) == 0) {
    const char *kv = line + 4;
    if (strncmp(kv, "MODE=", 5) == 0) {
      inputs.modeSelect = atoi(kv + 5);
    } else if (strncmp(kv, "BIOMASS=", 8) == 0) {
      inputs.biomassG = atof(kv + 8);
    } else if (strncmp(kv, "PWM=", 4) == 0) {
      inputs.pwmPercent = atoi(kv + 4);
      if (inputs.pwmPercent < 0) inputs.pwmPercent = 0;
      if (inputs.pwmPercent > 100) inputs.pwmPercent = 100;
    } else if (strncmp(kv, "GPS100=", 7) == 0) {
      inputs.gramsPerSec100 = atoi(kv + 7);
      if (inputs.gramsPerSec100 < 1) inputs.gramsPerSec100 = 1;
    } else if (strncmp(kv, "SIM_MODE=", 9) == 0) {
      inputs.simMode = (atoi(kv + 9) == 1);
    } else if (strncmp(kv, "SIM_TEMP=", 9) == 0) {
      inputs.simTempC = atof(kv + 9);
    } else if (strncmp(kv, "SIM_DIST=", 9) == 0) {
      inputs.simDistanceCm = atof(kv + 9);
    }
  } else if (strncmp(line, "CMD:", 4) == 0) {
    const char *cmd = line + 4;
    if (strcmp(cmd, "MANUAL") == 0) {
      requests.manualFeed = true;
    } else if (strcmp(cmd, "SIM_EVT") == 0) {
      requests.simEvent = true;
    } else if (strcmp(cmd, "SCHED_07") == 0) {
      requests.schedMorning = true;
    } else if (strcmp(cmd, "SCHED_17") == 0) {
      requests.schedEvening = true;
    }
  }
}

static void pollSerial() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n') {
      serialBuf[serialPos] = '\0';
      if (serialPos > 0) {
        handleSerialCommand(serialBuf);
      }
      serialPos = 0;
    } else if (c != '\r') {
      if (serialPos < sizeof(serialBuf) - 1) {
        serialBuf[serialPos++] = c;
      }
    }
  }
}

void setup() {
  Serial.begin(Config::SERIAL_BAUD);

  pinMode(Pins::ULTRASONIC_TRIG, OUTPUT);
  pinMode(Pins::ULTRASONIC_ECHO, INPUT);

  pinMode(Pins::MOTOR_L_PWM, OUTPUT);
  pinMode(Pins::MOTOR_R_PWM, OUTPUT);
  pinMode(Pins::MOTOR_L_EN, OUTPUT);
  pinMode(Pins::MOTOR_R_EN, OUTPUT);
  motorStop();

  pinMode(Pins::LED_STATUS, OUTPUT);
  pinMode(Pins::LED_LOW_FEED, OUTPUT);
  digitalWrite(Pins::LED_STATUS, LOW);
  digitalWrite(Pins::LED_LOW_FEED, LOW);

  pinMode(Pins::BTN_MANUAL, INPUT_PULLUP);

  valveServo.attach(Pins::SERVO);
  valveServo.write(Config::SERVO_CLOSE_DEG);

  Wire.begin();
  lcd.init();
  lcd.backlight();
  lcdSetLines("Booting...", "Please wait");

  dallas.begin();
}

void loop() {
  pollSerial();

  uint32_t now = millis();
  if (now - lastSampleMs >= Config::SAMPLE_INTERVAL_MS) {
    lastSampleMs = now;
    samplingTask();
  }

  handleButton();
  if (requests.manualFeed && feedState == IDLE) {
    requests.manualFeed = false;
    startFeedEvent("MANUAL");
  }
  if (requests.simEvent && feedState == IDLE) {
    requests.simEvent = false;
    startFeedEvent("SIM_EVT");
  }
  if (requests.schedMorning && feedState == IDLE) {
    requests.schedMorning = false;
    startFeedEvent("SCHED_07");
  }
  if (requests.schedEvening && feedState == IDLE) {
    requests.schedEvening = false;
    startFeedEvent("SCHED_17");
  }

  updateFeedState();
  digitalWrite(Pins::LED_STATUS, (feedState == IDLE) ? LOW : HIGH);
}
