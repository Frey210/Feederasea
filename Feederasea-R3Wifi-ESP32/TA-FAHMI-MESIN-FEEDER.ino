/***************************************************
 *  Automatic Feeder ESP32 + Blynk IoT
 *  Template  : Feederasea
 *  WiFi      : fariz / 12345678
 ***************************************************/

#include <Arduino.h>

#define BLYNK_TEMPLATE_ID   "TMPL6xwIRtIXO"
#define BLYNK_TEMPLATE_NAME "Feederasea"
#define BLYNK_AUTH_TOKEN    "iaeGFxiYGxAsVwIW_iG-2_-9tLYBp7iS"

#define BLYNK_PRINT Serial

#include <WiFi.h>
#include <BlynkSimpleEsp32.h>

#include <OneWire.h>
#include <DallasTemperature.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// ------------- WiFi -------------
char ssid[] = "fariz";
char pass[] = "12345678";

// ------------- Pin mapping -------------
const uint8_t PIN_DS18B20       = 4;
const uint8_t PIN_TRIG          = 12;
const uint8_t PIN_ECHO          = 14;
const uint8_t PIN_SERVO         = 26;
const uint8_t PIN_MOTOR_IN1     = 27;
const uint8_t PIN_MOTOR_IN2     = 25;
const uint8_t PIN_MOTOR_EN      = 33;  // PWM via analogWrite
const uint8_t PIN_BUTTON_MANUAL = 34;  // input pullup
const uint8_t PIN_LED_STATUS    = 2;   // LED status
const uint8_t PIN_LED_LOWFEED   = 15;  // LED low feed

// LCD I2C
const uint8_t LCD_ADDR = 0x27;
const uint8_t LCD_COLS = 16;
const uint8_t LCD_ROWS = 2;

// Hopper geometry & calibration
const float H_total_cm   = 40.0;
const float radius_cm    = 10.0;
float bulk_density_g_per_cm3 = 0.50;
float gramsPerSecAt100pct    = 6.0;

// Servo
const int SERVO_OPEN_ANGLE     = 45;
const int SERVO_CLOSE_ANGLE    = 0;
const unsigned long SERVO_STABILIZE_MS = 800;

// Keamanan
const unsigned long DISPENSE_TIMEOUT_MS = 12000UL;

// ---------- Objects ----------
OneWire oneWire(PIN_DS18B20);
DallasTemperature ds18b20(&oneWire);
Servo katupServo;
LiquidCrystal_I2C lcd(LCD_ADDR, LCD_COLS, LCD_ROWS);

// ---------- State ----------
bool  isDispensing      = false;
bool  useSimulationMode = true;     // SET ke false kalau mau pakai suhu real
float simulatedTemp     = 26.0;
float totalBiomass_g    = 4000.0;
int   operationMode     = 2;        // 0=A, 1=B, 2=C
int   blynk_pwm_pct     = 70;
float lastTempC         = 0.0;
float lastMassEst       = 0.0;

// ---------- Deklarasi fungsi ----------
void triggerDispenseEvent(const char* label);
float readTemperature();
float readUltrasonicCm();
float convertDistanceToMass(float distance_cm);
float calculateMassCommand(float tempC);
float computeRunTimeForMass(float mass_g, int pwm_pct);
void motorForwardPWM(int pwm_pct);
void motorStop();
void updateLCD(int mode, float tempC, float massEst);

// ================== BLYNK HANDLERS (INPUT) ==================
BLYNK_WRITE(V1) { simulatedTemp = param.asFloat(); }
BLYNK_WRITE(V2) { totalBiomass_g = param.asFloat(); }
BLYNK_WRITE(V3) { if (param.asInt() == 1) triggerDispenseEvent("BLYNK_MANUAL"); }
BLYNK_WRITE(V4) { operationMode   = param.asInt(); }
BLYNK_WRITE(V5) { blynk_pwm_pct   = param.asInt(); }
BLYNK_WRITE(V6) { gramsPerSecAt100pct = param.asFloat(); }
BLYNK_WRITE(V7) { if (param.asInt() == 1) triggerDispenseEvent("BLYNK_SIM_EVENT"); }

// ====================== SETUP ======================
void setup() {
  Serial.begin(115200);
  delay(200);

  Serial.println("=== Feeder ESP32 Start ===");

  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  pinMode(PIN_MOTOR_IN1, OUTPUT);
  pinMode(PIN_MOTOR_IN2, OUTPUT);
  pinMode(PIN_BUTTON_MANUAL, INPUT_PULLUP);
  pinMode(PIN_LED_STATUS, OUTPUT);
  pinMode(PIN_LED_LOWFEED, OUTPUT);
  pinMode(PIN_MOTOR_EN, OUTPUT);      // penting untuk analogWrite

  digitalWrite(PIN_MOTOR_IN1, LOW);
  digitalWrite(PIN_MOTOR_IN2, LOW);
  digitalWrite(PIN_LED_STATUS, LOW);
  digitalWrite(PIN_LED_LOWFEED, LOW);
  analogWrite(PIN_MOTOR_EN, 0);

  katupServo.attach(PIN_SERVO);
  katupServo.write(SERVO_CLOSE_ANGLE);

  Wire.begin(); // SDA=21, SCL=22
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0,0); lcd.print("Feeder Starting");
  lcd.setCursor(0,1); lcd.print("Init sensors...");

  ds18b20.begin();

  Serial.println("Connecting to WiFi & Blynk...");
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);

  lcd.clear();
  lcd.print("Feeder Ready");
  lcd.setCursor(0,1); lcd.print("Mode:");

  Serial.println("Setup complete");
}

// ====================== LOOP ======================
void loop() {
  Blynk.run();

  static int lastButton = HIGH;
  int b = digitalRead(PIN_BUTTON_MANUAL);
  if (lastButton == HIGH && b == LOW) {
    Serial.println("Manual button pressed");
    triggerDispenseEvent("LOCAL_MANUAL");
    delay(250);
  }
  lastButton = b;

  static unsigned long lastUpdate = 0;
  if (millis() - lastUpdate > 2000) {
    lastUpdate = millis();

    float tempC = readTemperature();
    if (useSimulationMode) tempC = simulatedTemp;
    lastTempC = tempC;

    float dcm = readUltrasonicCm();
    lastMassEst = convertDistanceToMass(dcm);

    updateLCD(operationMode, tempC, lastMassEst);

    Blynk.virtualWrite(V20, tempC);
    Blynk.virtualWrite(V21, lastMassEst);
    Blynk.virtualWrite(V22, totalBiomass_g);

    static bool ledState = false;
    ledState = !ledState;
    digitalWrite(PIN_LED_STATUS, ledState);
  }
}

// ================== FUNGSI UTAMA ==================

void triggerDispenseEvent(const char* label) {
  if (isDispensing) {
    Serial.println("Already dispensing. Trigger ignored.");
    return;
  }
  isDispensing = true;
  digitalWrite(PIN_LED_STATUS, HIGH);

  ds18b20.requestTemperatures();
  float tempC = ds18b20.getTempCByIndex(0);
  if (useSimulationMode) tempC = simulatedTemp;

  float dist_cm = readUltrasonicCm();
  float m_est_g = convertDistanceToMass(dist_cm);

  float mass_cmd_g = calculateMassCommand(tempC);

  Serial.printf(
    "EVENT_PRE,%lu,%s,mode=%d,temp=%.2f,dist=%.1f,m_est=%.1f,cmd=%.1f\n",
    millis(), label, operationMode, tempC, dist_cm, m_est_g, mass_cmd_g
  );

  Blynk.virtualWrite(V25, String(label)); // Last_Event

  if (operationMode == 2 && m_est_g < mass_cmd_g) {
    Serial.println("LOW_FEED - abort dispense");
    Blynk.logEvent("low_feed", "Feeder: LOW FEED - isi ulang pakan!");
    digitalWrite(PIN_LED_LOWFEED, HIGH);

    lcd.clear();
    lcd.print("LOW FEED!");
    lcd.setCursor(0,1); lcd.print("Refill required");
    delay(1500);

    isDispensing = false;
    digitalWrite(PIN_LED_STATUS, LOW);
    return;
  } else {
    digitalWrite(PIN_LED_LOWFEED, LOW);
  }

  int pwm_pct = blynk_pwm_pct;
  float run_s = computeRunTimeForMass(mass_cmd_g, pwm_pct);
  if (run_s * 1000.0 > DISPENSE_TIMEOUT_MS) {
    run_s = (float)DISPENSE_TIMEOUT_MS / 1000.0;
    Serial.println("run_time limited by timeout");
  }

  katupServo.write(SERVO_OPEN_ANGLE);
  delay(SERVO_STABILIZE_MS);

  motorForwardPWM(pwm_pct);
  unsigned long tstart = millis();
  while (millis() - tstart < (unsigned long)(run_s * 1000.0)) {
    delay(10);
  }
  motorStop();
  katupServo.write(SERVO_CLOSE_ANGLE);

  Serial.printf(
    "EVENT_POST,%lu,%s,cmd=%.2f,pwm=%d,run_s=%.2f\n",
    millis(), label, mass_cmd_g, pwm_pct, run_s
  );
  Blynk.logEvent("dispense_done", "Feeder: feeding selesai");

  Blynk.virtualWrite(V23, mass_cmd_g);
  Blynk.virtualWrite(V24, pwm_pct);

  lcd.clear();
  lcd.print("Dispense done");
  lcd.setCursor(0,1);
  lcd.print("Cmd g:");
  lcd.print((int)mass_cmd_g);

  delay(800);
  isDispensing = false;
  digitalWrite(PIN_LED_STATUS, LOW);
}

// Hitung massa pakan berdasarkan mode & suhu
float calculateMassCommand(float tempC) {
  float mass_cmd = 0.0;

  if (operationMode == 0) {
    mass_cmd = 50.0;
  } else if (operationMode == 1) {
    if (tempC >= 25.0 && tempC <= 37.0)
      mass_cmd = 0.03 * totalBiomass_g;
    else
      mass_cmd = 0.02 * totalBiomass_g;
  } else {
    if (tempC >= 25.0 && tempC <= 37.0)
      mass_cmd = 0.03 * totalBiomass_g;
    else
      mass_cmd = 0.02 * totalBiomass_g;
  }
  return mass_cmd;
}

// Kontrol motor dengan analogWrite
void motorForwardPWM(int pwm_pct) {
  digitalWrite(PIN_MOTOR_IN1, HIGH);
  digitalWrite(PIN_MOTOR_IN2, LOW);
  int duty = map(constrain(pwm_pct, 0, 100), 0, 100, 0, 255); // 8-bit duty
  analogWrite(PIN_MOTOR_EN, duty);
}

void motorStop() {
  digitalWrite(PIN_MOTOR_IN1, LOW);
  digitalWrite(PIN_MOTOR_IN2, LOW);
  analogWrite(PIN_MOTOR_EN, 0);
}

// Hitung waktu nyala motor (detik) untuk massa tertentu
float computeRunTimeForMass(float mass_g, int pwm_pct) {
  if (pwm_pct <= 0) return 0.0;
  float speedFactor = (float)pwm_pct / 100.0;
  float gramsPerSec = gramsPerSecAt100pct * speedFactor;
  if (gramsPerSec <= 0.0001) return 0.0;
  float t = mass_g / gramsPerSec;
  Serial.printf("ComputeRunTime: mass=%.2fg pwm=%d -> t=%.2fs\n",
                mass_g, pwm_pct, t);
  return t;
}

// Baca suhu dari DS18B20
float readTemperature() {
  ds18b20.requestTemperatures();
  float t = ds18b20.getTempCByIndex(0);
  if (t == DEVICE_DISCONNECTED_C) {
    Serial.println("DS18B20 disconnected!");
    return NAN;
  }
  return t;
}

// Baca jarak dari HC-SR04 (cm)
float readUltrasonicCm() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);

  unsigned long duration = pulseIn(PIN_ECHO, HIGH, 30000);
  if (duration == 0) {
    Serial.println("Ultrasonic timeout");
    return H_total_cm;
  }
  float distance_cm = (float)duration / 58.0;
  if (distance_cm < 0.0)        distance_cm = 0.0;
  if (distance_cm > H_total_cm) distance_cm = H_total_cm;
  return distance_cm;
}

// Konversi jarak → massa pakan (g)
float convertDistanceToMass(float distance_cm) {
  float h_cm = H_total_cm - distance_cm;
  if (h_cm < 0.0) h_cm = 0.0;

  float area_cm2   = 3.14159265 * radius_cm * radius_cm;
  float volume_cm3 = area_cm2 * h_cm;
  float mass_g     = volume_cm3 * bulk_density_g_per_cm3;

  Serial.printf("EstimateMass: dist=%.2f cm h=%.2f cm vol=%.2f cm3 mass=%.2f g\n",
                distance_cm, h_cm, volume_cm3, mass_g);
  return mass_g;
}

void updateLCD(int mode, float tempC, float massEst) {
  lcd.clear();
  lcd.setCursor(0,0);

  String modeName = (mode == 0) ? "A:TIME" :
                    (mode == 1) ? "B:TEMP" : "C:COMBO";
  lcd.print(modeName);
  lcd.print(" T:");
  if (!isnan(tempC)) lcd.print((int)tempC);
  else               lcd.print("--");

  lcd.setCursor(0,1);
  lcd.print("Rem(g):");
  lcd.print((int)massEst);
}
