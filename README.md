# Feederasea (UNO + ESP8266 + Blynk)

Firmware feeder otomatis dengan arsitektur dual-MCU:
- Arduino UNO menangani sensor dan aktuator.
- ESP8266 menangani WiFi, Blynk, dan jadwal feeding.

Dokumen ini menjelaskan komponen, wiring, GPIO, protokol serial, serta cara upload firmware.

---

## 1) Ringkasan Fitur

- Blynk IoT untuk kontrol dan telemetry.
- DS18B20 membaca suhu air.
- HC-SR04 membaca jarak pakan di hopper, dikonversi ke massa pakan (g).
- Servo MG996R membuka dan menutup katup pakan.
- Motor BTS7960 menggerakkan pakan sesuai durasi terhitung.
- LCD I2C 16x2 menampilkan mode, suhu, jarak, dan estimasi pakan.
- State machine non-blocking untuk dispensing.
- Mode A/B/C sesuai suhu, biomassa, dan jadwal.

---

## 2) Struktur Project

```
Feederasea/
|-- firmware/
|   |-- uno/feeder_uno.ino
|   |-- esp8266/feeder_esp8266.ino
|-- include/
|   |-- secrets.h
|-- src/
|   |-- main.cpp   (legacy ESP32 build)
|-- README.md
```

---

## 3) Daftar Komponen Lengkap

- Board UNO + ESP8266 (UNO R3 ATmega328P + NodeMCU ESP8266, CH340G)
- Sensor suhu DS18B20 + resistor 4.7k pull-up
- Sensor jarak HC-SR04 + level shifter (atau divider) untuk ECHO 3.3V
- Driver motor BTS7960
- Motor DC + piringan pelontar pakan
- Servo MG996R
- LCD I2C 16x2 (alamat umum 0x27)
- Push button manual + resistor pull-up eksternal (opsional jika INPUT_PULLUP)
- LED status + resistor, LED low feed + resistor
- Power supply 12V (motor) dan 5V (ESP32/ESP8266, servo, sensor, LCD)
- Step-down 12V ke 5V (contoh LM2596)
- Kabel, terminal, dan konektor

---

## 4) Wiring Diagram (Text)

### Power
- 12V: masuk ke BTS7960 untuk motor
- 5V: UNO, servo, DS18B20, HC-SR04, LCD
- Semua GND disatukan (common ground)

### DS18B20 (UNO)
- VCC -> 3.3V
- GND -> GND
- DATA -> D4 + resistor 4.7k ke 3.3V

### HC-SR04 (UNO)
- VCC -> 5V
- GND -> GND
- TRIG -> D12
- ECHO -> level-shift -> D11

### BTS7960 (UNO)
- VCC -> 5V (logic)
- GND -> GND
- L_EN -> D7
- R_EN -> D8
- LPWM -> D5
- RPWM -> D6
- MOTOR+/- -> output driver ke motor DC

### Servo MG996R (UNO)
- VCC -> 5V (supply terpisah lebih baik)
- GND -> GND
- SIG -> D9

### LCD I2C (UNO)
- VCC -> 5V
- GND -> GND
- SDA -> A4
- SCL -> A5

### LED dan Tombol (UNO)
- LED Status: D13 -> resistor -> GND
- LED Low Feed: D10 -> resistor -> GND
- Tombol manual: satu kaki ke GND, satu kaki ke D2 (gunakan INPUT_PULLUP)

### UART UNO <-> ESP8266
- Gunakan koneksi serial bawaan board (D0/D1 UNO ke RX/TX ESP) melalui DIP switch.
- Ikuti label DIP pada board untuk mode "UNO<->ESP" saat runtime.
- Periksa dokumentasi board untuk posisi switch yang tepat.

---

## 5) List GPIO (UNO)

- D2  : Manual button (INPUT_PULLUP)
- D4  : DS18B20 data
- D5  : BTS7960 LPWM
- D6  : BTS7960 RPWM
- D7  : BTS7960 L_EN
- D8  : BTS7960 R_EN
- D9  : Servo MG996R signal
- D10 : LED low feed
- D11 : HC-SR04 ECHO (level-shift)
- D12 : HC-SR04 TRIG
- D13 : LED status
- A4  : I2C SDA (LCD)
- A5  : I2C SCL (LCD)

---

## 6) Konfigurasi Blynk

Template: `Feederasea`
Auth Token: lihat `include/secrets.h`.

### Datastream (Input dari App)
- V1  `Sim_Temp` (Double, C)
- V2  `Biomass` (Double, g)
- V3  `Manual_Feed` (Integer, 0/1)
- V4  `Mode_Select` (Integer, 0..2)
- V5  `PWM_Percent` (Integer, 0..100)
- V6  `GramPerSec_100` (Integer, 1..20)
- V7  `Sim_Event` (Integer, 0/1)
- V8  `Test_In` (Double, cm) -> simulated distance

Catatan:
- V1 dan V8 hanya dipakai jika V7 (Sim_Event) aktif.

### Telemetry (Output ke App)
- V20 `TempC` (suhu real dari DS18B20)
- V21 `Feed_Remaining` (massa pakan tersisa)
- V22 `Biomass_Out`
- V23 `Last_Cmd_Grams`
- V24 `Last_PWM`
- V25 `Last_Event`

---

## 7) Protokol Serial (UNO <-> ESP8266)

### Command dari ESP ke UNO
- `SET:MODE=0|1|2`
- `SET:BIOMASS=xxxx`
- `SET:PWM=0..100`
- `SET:GPS100=1..20`
- `SET:SIM_MODE=0|1`
- `SET:SIM_TEMP=xx.xx`
- `SET:SIM_DIST=xx.xx`
- `CMD:MANUAL`
- `CMD:SIM_EVT`
- `CMD:SCHED_07`
- `CMD:SCHED_17`

### Telemetry dari UNO ke ESP
```
TELEM,T=xx.xx,TR=yy.yy,DIST=zz.z,FEED=nnn,MODE=m,STATE=s,EVENT=LABEL,BIOMASS=bbbb,CMD=ccc,PWM=pp,SIM=0|1
```

---

## 8) Cara Upload Firmware

### A) Upload UNO
- Buka `firmware/uno/feeder_uno.ino` dengan Arduino IDE.
- Board: Arduino Uno.
- Baud serial: 9600.
- Atur DIP switch ke mode upload UNO.
- Upload.

### B) Upload ESP8266
- Buka `firmware/esp8266/feeder_esp8266.ino` dengan Arduino IDE.
- Board: NodeMCU 1.0 (ESP-12E) atau yang setara.
- Install library: Blynk.
- Isi WiFi dan token di `include/secrets.h`.
- Atur DIP switch ke mode upload ESP.
- Upload.

### C) Run Mode
- Atur DIP switch ke mode runtime UNO<->ESP.
- Reset board.

---

## 9) Cara Kerja Utama

- UNO membaca sensor, menghitung massa pakan, dan menjalankan motor/servo.
- ESP8266 menerima data telemetry dari UNO dan mengirim ke Blynk.
- Jadwal feeding (Mode C) dihitung di ESP8266 menggunakan NTP.
- Simulasi (V7) mengaktifkan penggunaan V1/V8; saat V7=0 sensor real dipakai.

---

## 10) Troubleshooting Ringkas

- LCD stuck "Feeding...": pastikan tombol manual tidak floating, gunakan INPUT_PULLUP dan tombol ke GND.
- V1/V8 tidak berpengaruh: pastikan V7 aktif.
- Tidak ada data di Blynk: pastikan DIP switch runtime menghubungkan UNO<->ESP.
- Motor tidak jalan: cek power 12V dan wiring BTS7960.

---

## 11) Catatan Safety

- Jangan menyalakan motor tanpa beban terlalu lama.
- Pastikan semua ground tersambung.
- HC-SR04 gunakan level shift pada ECHO.
