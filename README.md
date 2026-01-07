# Feederasea (ESP32 + Blynk + PlatformIO)

Firmware feeder otomatis berbasis ESP32 dengan Blynk IoT, sensor suhu DS18B20, sensor jarak HC-SR04, motor BTS7960, servo katup, dan LCD I2C 16x2.

Dokumen ini menjelaskan konfigurasi, pin mapping, cara kerja, mode operasi, dan tutorial penggunaan.

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

## 2) Struktur Project (PlatformIO)

```
Feederasea/
|-- platformio.ini
|-- include/
|   |-- secrets.h
|-- src/
|   |-- main.cpp
|-- README.md
```

---

## 3) Hardware dan Pin Mapping

### Sensor dan Aktuator
- DS18B20: GPIO4
- HC-SR04: TRIG GPIO12, ECHO GPIO14 (ECHO harus level-shift ke 3.3V)
- Servo MG996R: GPIO26
- BTS7960:
  - LPWM = GPIO32
  - RPWM = GPIO33
  - L_EN = GPIO27
  - R_EN = GPIO25

Catatan BTS7960:
- Paper menyebut L298N (IN1/IN2/ENA). Pada firmware ini driver tetap BTS7960.
- Gunakan GPIO27 dan GPIO25 untuk L_EN dan R_EN.
- Gunakan GPIO32 dan GPIO33 untuk LPWM dan RPWM.

### LCD I2C
- SDA = GPIO21
- SCL = GPIO22
- Address default = 0x27

### LED dan Tombol
- Status LED = GPIO2
- LOW_FEED LED = GPIO15
- Manual Button = GPIO34 (input-only, aktif LOW, butuh pull-up eksternal)

---

## 4) Konfigurasi Blynk

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

Pastikan datastream V1..V8 writeable (Input atau Input/Output) agar ESP32 menerima perubahan dari app.

### Telemetry (Output ke App)
- V20 `TempC` (suhu real dari DS18B20)
- V21 `Feed_Remaining` (massa pakan tersisa)
- V22 `Biomass_Out`
- V23 `Last_Cmd_Grams`
- V24 `Last_PWM`
- V25 `Last_Event`

---

## 5) Cara Kerja Utama

Firmware berjalan dengan loop non-blocking menggunakan `Blynk.run()` dan `BlynkTimer`.

### Sampling (tiap 2 detik)
1. Baca DS18B20 (suhu real).
2. Baca jarak HC-SR04 (atau gunakan nilai V8 jika simulasi jarak aktif).
3. Konversi jarak ke massa pakan (model silinder).
4. Update LCD dan kirim telemetry ke Blynk.

### Estimasi Massa (Hopper Silinder)
- `heightFilled = H_TOTAL_CM - distance_cm`
- `volume = pi * radius_cm^2 * heightFilled`
- `mass = volume * bulk_density`

Parameter dapat dikalibrasi di `src/main.cpp`:
- `H_TOTAL_CM`
- `RADIUS_CM`
- `BULK_DENSITY_G_PER_CM3`

---

## 6) Mode Operasi (A/B/C)

### Mode A (0)
- Komando pakan tetap: 50 g per event.

### Mode B (1)
- Komando pakan berdasarkan suhu:
  - 25 C sampai 37 C: 3% dari `Biomass`
  - di luar rentang: 2% dari `Biomass`

### Mode C (2)
- Jadwal feeding pada 07:00 dan 17:00 (WITA, NTP).
- Rumus komando sama dengan Mode B.
- Guard agar tidak double trigger dalam hari yang sama.
- Jika `Feed_Remaining < Commanded + 1g` maka abort, LED LOW_FEED menyala, log event `low_feed`.

---

## 7) State Machine Feeding

Urutan state:
- `IDLE`
- `PRECHECK`
- `SERVO_OPENING`
- `MOTOR_RUNNING`
- `SERVO_CLOSING`
- `POSTLOG`
- `ABORT_LOW_FEED`

Waktu utama:
- Servo open settle: 800 ms
- Servo close settle: 600 ms
- Motor runtime: dihitung dari grams per second (dibatasi max 12 s)

Rumus runtime:
```
GramsPerSec = GramPerSecAt100pct * (PWM_Percent / 100.0)
Runtime_s = Commanded_grams / GramsPerSec
Clamp runtime max 12s
```

---

## 8) Tutorial Penggunaan

### Langkah 1: Wiring
- Sambungkan komponen sesuai pin mapping di atas.
- Pastikan semua GND tersambung menjadi satu.
- ECHO HC-SR04 harus level-shift ke 3.3V.

### Langkah 2: Setup WiFi dan Blynk
- Buka `include/secrets.h`.
- Isi `BLYNK_AUTH_TOKEN`, `WIFI_SSID`, dan `WIFI_PASS`.

### Langkah 3: Build dan Upload
```
platformio run
platformio run -t upload
platformio device monitor -b 115200
```

### Langkah 4: Konfigurasi Blynk App
- Buat datastream V1..V8 dan V20..V25 sesuai list di atas.
- Pastikan V1..V8 writeable.
- Atur widget slider atau input sesuai kebutuhan.

### Langkah 5: Operasi Harian
- Pilih mode dengan V4 (0, 1, 2).
- Atur `Biomass` di V2.
- Atur `PWM_Percent` di V5 dan `GramPerSec_100` di V6 untuk kalibrasi laju.
- Tekan `Manual_Feed` (V3) untuk feed manual.
- Tekan `Sim_Event` (V7) untuk simulasi event.

### Langkah 6: Simulasi Sensor
- V1 `Sim_Temp` untuk simulasi suhu.
- V8 `Test_In` untuk simulasi jarak pakan (cm).

---

## 9) Troubleshooting Ringkas

### 9.1 Blynk Input Tidak Masuk
- Pastikan V1..V8 writeable.
- Pastikan device menggunakan auth token yang benar.
- Cek Serial log: `Blynk Vx ...` harus muncul saat nilai diubah.

### 9.2 Feed Remaining Selalu 0
- Pastikan HC-SR04 membaca jarak (cek `Dist` di serial).
- Set `H_TOTAL_CM` sesuai jarak kosong.
- Pastikan echo level-shift dan sensor diberi 5V.

### 9.3 Suhu Real NaN
- DS18B20 belum terbaca, cek wiring dan resistor pull-up 4.7k ke 3.3V.

---

## 10) Diagram Alur (Mermaid)

### 10.1 Flow Sampling dan Telemetry
```mermaid
flowchart TD
  A[Timer 2s] --> B[Read DS18B20]
  B --> C[Read HC-SR04]
  C --> D[Apply sim distance V8]
  D --> E[Compute Feed Mass]
  E --> F[Update LCD]
  F --> G[Send Telemetry to Blynk]
```

### 10.2 Feeding State Machine
```mermaid
stateDiagram-v2
  [*] --> IDLE
  IDLE --> PRECHECK: trigger feed
  PRECHECK --> ABORT_LOW_FEED: Mode C and feed insufficient
  PRECHECK --> SERVO_OPENING: ok
  SERVO_OPENING --> MOTOR_RUNNING: after 800ms
  MOTOR_RUNNING --> SERVO_CLOSING: runtime done
  SERVO_CLOSING --> POSTLOG: after 600ms
  POSTLOG --> IDLE
  ABORT_LOW_FEED --> IDLE: after 3s
```

### 10.3 Mode Selection Logic
```mermaid
flowchart TD
  A[Trigger Event] --> B{Mode Select}
  B -->|A| C[Command 50g]
  B -->|B| D[Command 2 or 3 percent Biomass]
  B -->|C| E[Schedule 07:00 17:00 and Temp Rule]
  C --> F[Compute Runtime]
  D --> F
  E --> F
```

---

## 11) Catatan Safety

- Jangan menyalakan motor tanpa beban terlalu lama.
- Pastikan ground semua modul disatukan.
- HC-SR04 sebaiknya diberi 5V dan echo di level-shift ke 3.3V.
