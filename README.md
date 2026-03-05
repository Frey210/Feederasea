# Feederasea (ESP32 + Arduino UNO)

Project ini adalah feeder otomatis berbasis IoT. ESP32 menangani WiFi/Blynk, jadwal, dan komunikasi UART. Arduino UNO menangani sensor, perhitungan massa pakan, motor/servo, dan LCD.

---

## 1) Struktur Folder

```
Feederasea/
+- Feederasea-R3Wifi-ESP32/   # Firmware ESP32 (Blynk + WiFi + schedule + UART)
+- Feederasea-R3Wifi-UNO/     # Firmware UNO (sensor + aktuator + LCD)
```

---

## 2) Komponen Utama

- ESP32 DevKit 38-pin
- Arduino UNO R3
- DS18B20 + resistor 4.7k (pull-up)
- VL53L1X (ToF, I2C)
- Driver motor BTS7960 + motor DC
- Servo (contoh MG996R)
- LCD I2C 16x2 (alamat umum 0x27)
- Tombol manual, LED status, LED low feed
- Power 12V (motor) + 5V (board/sensor), step-down 12V->5V

---

## 3) Wiring Ringkas

### UART ESP32 <-> UNO
- UNO D0 (RX)  <- ESP32 TX0 (GPIO1)
- UNO D1 (TX)  -> ESP32 RX0 (GPIO3) **wajib level shift 5V -> 3.3V**
- GND harus common

### Sensor/Actuator di UNO (lihat detail di `Feederasea-R3Wifi-UNO/README.md`)
- DS18B20 -> D4
- VL53L1X -> SDA A4, SCL A5 (I2C shared dengan LCD)
- Servo -> D9
- BTS7960: LPWM D5, RPWM D6, L_EN D7, R_EN D8
- LCD I2C -> A4/A5
- Button -> D2 (INPUT_PULLUP)
- LED status D13, LED low feed D10

---

## 4) Setup dan Upload (PlatformIO)

### ESP32
1. Buka folder `Feederasea-R3Wifi-ESP32`.
2. Isi WiFi dan token di `Feederasea-R3Wifi-ESP32/include/secrets.h`.
3. Build & upload:
```
platformio run
platformio run -t upload
platformio device monitor -b 9600
```

### UNO
1. Buka folder `Feederasea-R3Wifi-UNO`.
2. Build & upload:
```
platformio run
platformio run -t upload
platformio device monitor -b 9600
```

Catatan: Saat upload ESP32, lepaskan UART ke UNO agar tidak mengganggu flashing.

---

## 5) Blynk Datastream

### Input (dari app)
- V1  `Sim_Temp` (C)
- V2  `Biomass` (g)
- V3  `Manual_Feed` (0/1)
- V4  `Mode_Select` (0..3)
- V5  `PWM_Percent` (0..100)
- V6  `GramPerSec_100` (1..20)
- V7  `Sim_Event` (0/1)
- V8  `Test_In` (cm)

### Output (ke app)
- V20 `TempC`
- V21 `Feed_Remaining`
- V22 `Biomass_Out`
- V23 `Last_Cmd_Grams`
- V24 `Last_PWM`
- V25 `Last_Event` (log status, cocok untuk Widget Terminal)

---

## 6) Mode Operasi

- Mode A (0): komando pakan tetap 50g.
- Mode B (1): 25-37C = 3% biomassa, di luar = 2%.
- Mode C (2): jadwal otomatis jam 07:00 dan 17:00 (WITA), rumus sama dengan Mode B.
- Mode D (3): jadwal otomatis tetap jam 08:00, 13:30, 19:00 dengan gram berbasis suhu tetap:
  - 4C sampai <24C -> 30g
  - 24C sampai <32C -> 40g
  - 32C sampai 37C -> 50g
  Sebelum dispense, sistem cek jarak pakan. Jika jarak <= 2.0 cm, event dibatalkan dan tampil peringatan "Pakan Habis" + buzzer.

Catatan Mode D:
- Simulasi ON: trigger manual tetap boleh kapan saja (tanpa menunggu jadwal), perhitungan gram memakai suhu simulasi.
- Simulasi OFF: auto feed hanya lewat jadwal Mode D, tombol manual tetap berfungsi dan tetap memakai logika gram Mode D.

---

## 7) Flow Diagram (Mermaid)

```mermaid
flowchart TD
  A[ESP32 Boot] --> B[WiFi + Blynk Connect]
  B --> C[Sync Datastream Input]
  C --> D[Send Config -> UNO]
  D --> E[UNO Read Sensor + Update LCD]
  E --> F[UNO Send Telemetry -> ESP32]
  F --> G[ESP32 Update Blynk Widgets]
  G --> H[Check Schedule Mode C]
  H --> D

  I[User Action in Blynk] --> D
  J[Manual Button UNO] --> K[Feed Event]
  K --> F
```

---

## 8) Algoritma Estimasi Sisa Pakan (Jarak -> Gram)

Estimasi `Feed_Remaining` di UNO dihitung dari jarak VL53L1X menggunakan pipeline matematis berikut.

### A) Definisi Variabel
- `d_raw(k)` : jarak mentah pembacaan ke-`k` (cm), hasil rata-rata beberapa sampel valid.
- `d_cal(k)` : jarak setelah koreksi skala/offset.
- `d_f(k)` : jarak terfilter (cm), dipakai untuk perhitungan massa.
- `m(k)` : estimasi massa pakan (gram).
- `alpha_fast`, `alpha_slow` : koefisien filter cepat/lambat.
- `delta_th` : ambang perubahan untuk memilih filter cepat/lambat.
- `jitter_band` : deadband perubahan minimum (cm).
- `d_empty` : jarak saat hopper kosong (`DIST_EMPTY_CM`).
- `zero_band` : toleransi area kosong (`EMPTY_ZERO_BAND_CM`).

### B) Akuisisi Sensor
Untuk setiap siklus, UNO mengambil `N` sampel valid dari VL53L1X, lalu:

`d_raw(k) = (1 / N) * sum(d_i), i = 1..N`

Jika tidak ada sampel valid (timeout), firmware menahan nilai terakhir:

`d_raw(k) = d_f(k-1)`

### C) Koreksi Skala dan Offset
Jarak hasil sensor dikoreksi:

`d_cal(k) = DIST_SCALE * d_raw(k) + DIST_OFFSET_CM`

### D) Filter Adaptif (Responsif tapi Stabil)
Hitung besar perubahan:

`Delta(k) = |d_cal(k) - d_f(k-1)|`

Pilih koefisien:
- jika `Delta(k) >= DIST_FAST_THRESHOLD_CM` -> `alpha = DIST_FAST_ALPHA`
- jika `Delta(k) < DIST_FAST_THRESHOLD_CM` -> `alpha = DIST_SLOW_ALPHA`

Update filter eksponensial:

`d_tmp(k) = alpha * d_cal(k) + (1 - alpha) * d_f(k-1)`

Deadband jitter:
- jika `|d_tmp(k) - d_f(k-1)| < DIST_JITTER_CM` -> `d_f(k) = d_f(k-1)`
- selain itu -> `d_f(k) = d_tmp(k)`

### E) Konversi Jarak ke Massa (Piecewise Linear)
Gunakan titik kalibrasi `(d_j, m_j)` dari hasil timbangan nyata, urut dari kosong ke penuh.
Untuk `d_f(k)` yang berada di antara `d_hi` dan `d_lo`:

`t = (d_hi - d_f(k)) / (d_hi - d_lo)`

`m(k) = m_hi + t * (m_lo - m_hi)`

Ini adalah interpolasi linear per-segmen (piecewise).

### F) Zero-Band pada Kondisi Kosong
Ambang kosong:

`d_zero_th = d_empty - zero_band`

Jika `d_f(k) >= d_zero_th`, maka dipaksa:

`m(k) = 0`

Selain itu, firmware juga menerapkan cutoff massa kecil (mis. `< 20 g`) ke nol untuk menahan noise residual.

### G) Contoh Hitung Singkat
Misal:
- `d_f(k-1) = 40.0 cm`
- `d_cal(k) = 39.2 cm`
- `DIST_FAST_THRESHOLD_CM = 1.0`
- `DIST_SLOW_ALPHA = 0.25`

Maka:
- `Delta = 0.8 < 1.0` -> pakai alpha lambat `0.25`
- `d_tmp = 0.25*39.2 + 0.75*40.0 = 39.8 cm`
- Jika lolos deadband, `d_f(k) = 39.8 cm`
- Jika `d_empty = 40.0` dan `zero_band = 1.2`, maka `d_zero_th = 38.8`
- Karena `39.8 >= 38.8`, hasil akhir `m(k) = 0 g`

### H) Parameter Tuning (File UNO)
Lokasi: `Feederasea-R3Wifi-UNO/src/main.ino`
- Zero point: `DIST_EMPTY_CM`, `EMPTY_ZERO_BAND_CM`
- Respons filter: `DIST_SLOW_ALPHA`, `DIST_FAST_ALPHA`, `DIST_FAST_THRESHOLD_CM`, `DIST_JITTER_CM`
- Mapping gram: titik kalibrasi di `estimateFeedMassG()`
- Koreksi sensor: `DIST_SCALE`, `DIST_OFFSET_CM`

---

## 9) Panduan Penggunaan (User Guide)

### A) Mode Feeding
- **Mode A (0)**: Pakan tetap 50g per event.
- **Mode B (1)**: Pakan berdasarkan suhu (25-37C = 3% biomassa, di luar = 2%).
- **Mode C (2)**: Sama dengan Mode B, tetapi otomatis pada 07:00 dan 17:00 (WITA).
- **Mode D (3)**: Jadwal 08:00, 13:30, 19:00 dengan fixed gram by suhu:
  - 4C sampai <24C = 30g
  - 24C sampai <32C = 40g
  - 32C sampai 37C = 50g
  Event dibatalkan jika jarak pakan <= 2.0 cm (buzzer + LCD "Pakan Habis").

### B) Interface Kontrol (Blynk Input)
- **V3 Manual_Feed**: Tombol manual untuk men-trigger pakan.
- **V4 Mode_Select**: Pilih mode A/B/C (0/1/2).
- **V4 Mode_Select**: Pilih mode A/B/C/D (0/1/2/3).
- **V5 PWM_Percent**: Kecepatan motor (%).
- **V6 GramPerSec_100**: Kalibrasi laju pakan pada PWM 100%.
- **V2 Biomass**: Total biomassa ikan (gram), dipakai di Mode B/C.
- **V7 Sim_Event**: Aktifkan simulasi dan trigger event simulasi.
- **V1 Sim_Temp** dan **V8 Test_In**: Input simulasi suhu dan jarak (aktif saat V7=1).

### C) Interface Monitoring (Blynk Output)
- **V20 TempC**: Suhu efektif (real/simulasi).
- **V21 Feed_Remaining**: Estimasi sisa pakan di hopper.
- **V22 Biomass_Out**: Biomassa saat ini (echo dari input).
- **V23 Last_Cmd_Grams**: Gram perintah terakhir.
- **V24 Last_PWM**: PWM terakhir yang dipakai.
- **V25 Last_Event**: Log status ringkas (disarankan widget Terminal).

### D) Alur Operasi Singkat
1. Nyalakan alat, ESP32 connect WiFi dan Blynk.
2. Atur biomassa (V2), mode (V4), dan kalibrasi (V6, V5).
3. Untuk manual feed, tekan V3.
4. Untuk mode jadwal, pilih Mode C dan biarkan sistem otomatis.
   Untuk Mode D, sistem berjalan pada jadwal 08:00 / 13:30 / 19:00 dengan precheck jarak pakan.
5. Pantau V20-V25 untuk status dan hasil event.

---

## 10) Protokol Serial (ESP32 <-> UNO)

ESP32 -> UNO:
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

UNO -> ESP32:
```
TELEM,T=xx.xx,TR=yy.yy,DIST=zz.z,FEED=nnn,MODE=m,STATE=s,EVENT=LABEL,BIOMASS=bbbb,CMD=ccc,PWM=pp,SIM=0|1
```

---

## 11) Troubleshooting

- Blynk nilai 0: cek UART UNO<->ESP32 (RX/TX, GND, level shift), baud 9600.
- `STALE` di V25: telemetry dari UNO tidak masuk atau terputus.
- LCD stuck "Feeding...": cek tombol manual (gunakan INPUT_PULLUP).
- Motor tidak jalan: cek supply 12V dan wiring BTS7960.

---

## 12) Catatan Keamanan

- Jangan commit `include/secrets.h` ke repo publik jika berisi token/SSID asli.
- Gunakan level shifting untuk sinyal 5V -> 3.3V (minimal UNO TX -> ESP32 RX).
- Jangan menyalakan motor tanpa beban terlalu lama.
- Pastikan semua ground tersambung (common ground).

