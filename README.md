# ESP32 Tank — Arduino IDE

Xe tank ESP32 + TB6612FNG + 2 motor JGB37 encoder + ThingsBoard (manual / auto).

## Cài đặt Arduino IDE

1. Cài [Arduino IDE 2.x](https://www.arduino.cc/en/software)
2. **Board Manager:** cài **esp32** by Espressif (v3.x khuyến nghị)
3. **Library Manager** — cài 2 thư viện:
   - `PubSubClient` — Nick O'Leary
   - `ArduinoJson` — Benoit Blanchon (v7)
4. Mở file `esp32-tank-jbg37.ino` (Arduino IDE sẽ nạp cả folder sketch)

> **Encoder** dùng ngắt `RISING` giống code xe cân bằng — **không cần** thư viện ESP32Encoder.

## Cấu trúc sketch

```
esp32-tank-jbg37/
├── esp32-tank-jbg37.ino   ← setup/loop, WiFi, MQTT
├── config.h               ← chân GPIO, WiFi, ThingsBoard
├── motor_driver.h/.cpp    ← TB6612FNG
├── odometry.h/.cpp        ← encoder (ngắt, giống xe cân bằng)
└── mission.h/.cpp         ← manual/auto, hàng đợi di chuyển
```

## Sơ đồ chân

| Thiết bị | GPIO |
|----------|------|
| Motor phải PWMA / AIN1 / AIN2 | 27 / 26 / 25 |
| Motor trái PWMB / BIN1 / BIN2 | 13 / 33 / 32 |
| TB6612 STBY | 3.3V |
| Enc phải C1 / C2 | 12 / 14 |
| Enc trái C1 / C2 | 35 / 34 (cần trở kéo 10k) |
| Buzzer | 15 |
| Gas sensor | 36 |

## ThingsBoard — Shared Attributes

| Key | Mô tả |
|-----|--------|
| `mode` | `"manual"` hoặc `"auto"` |
| `x`, `y` | Điều khiển manual (-100..100), tank drive |
| `duration` | ms tự dừng (manual) |
| `stop` | `true` → dừng ngay |
| `auto_enabled` | bật lịch auto |
| `target_x`, `target_y` | đích (0..150 cm) |
| `run_at_epoch` | Unix time chạy (0 = chạy ngay) |
| `auto_start_now` | `true` → bắt đầu mission |
| `counts_per_cm` | hiệu chỉnh encoder (mặc định 55) |
| `turn_90_counts` | encoder khi xoay 90° (mặc định 620) |
| `scan_360_ms` | thời gian quét gas 360° (mặc định 1700) |

**Manual** — ví dụ tiến:
```json
{"mode":"manual","x":0,"y":80,"duration":2000}
```

**Auto** — đi tới (100, 80) cm, quét gas, về (0,0):
```json
{"mode":"auto","auto_enabled":true,"target_x":100,"target_y":80,"auto_start_now":true}
```

Bản đồ: **150 × 150 cm**. Gốc (0,0) = vị trí xuất phát.

## Hiệu chỉnh encoder

1. Cho xe đi thẳng 50 cm, đọc `enc_l`/`enc_r` trên telemetry
2. `counts_per_cm = enc_avg / 50`
3. Xoay 90° tại chỗ, đọc counts → gán `turn_90_counts`

Nếu motor/encoder quay ngược: sửa `MOTOR_R_SIGN`, `MOTOR_L_SIGN`, `ENC_R_SIGN`, `ENC_L_SIGN` trong `config.h`.

## Board settings

- Board: **ESP32 Dev Module**
- Upload speed: 921600
- Partition: **Default 4MB** (hoặc Huge APP nếu compile báo thiếu flash)

## Test motor trước (khuyến nghị)

Mở sketch **`motor_test/motor_test.ino`** → Upload → Serial Monitor 115200.

| Phím | Chức năng |
|------|-----------|
| `a` | Chuỗi test tự động (tiến/lùi/từng bánh/xoay) |
| `1`–`8` | Điều khiển từng motor / xoay |
| `0` | Dừng |
| `e` | In encoder |
| `r` | Reset encoder |
| `+`/`-` | Tăng/giảm PWM |

Nếu encoder đếm ngược hoặc 1 bánh quay sai chiều → chỉnh `MOTOR_*_SIGN` / `ENC_*_SIGN` trong `config.h` rồi nạp lại firmware chính.
