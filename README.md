# ESP32 Tank JGB37 — Wiring & Pin Configuration

Detailed wiring guide for this tank project using **ESP32 Dev Module**, **L298N** motor driver, and two **JGB37** gear motors with encoders.

---

## 1. Power Supply (recommended)

```
3S Li-ion (3 × 4.2 V, ~12.6 V full)
        │
        ▼
    XL4016 buck (step-down)
        │
        ▼
    LM2596 buck (final step-down)
        │
        ├── ESP32 (5 V via VIN / 5V pin, or USB 5 V for programming)
        └── L298N (logic + motor supply)
```

### Recommended voltages

| Rail | Voltage | Notes |
|------|---------|-------|
| LM2596 output | **5.0 V** | ESP32 logic |
| L298N motor supply | **6–12 V** | JGB37 motors; jumper ON nếu dùng cùng nguồn 5V (chỉ test nhẹ) |
| Encoder VCC | **3.3 V** | Cả hai encoder |

### Critical rules

1. **Common ground** — nối chung GND pin nguồn, ESP32, L298N, encoder.
2. **Tụ lọc** — 470–1000 µF gần nguồn motor; 100 µF + 0.1 µF gần ESP32.
3. **Không cấp 12 V trực tiếp vào ESP32** — hạ áp xuống 5 V trước.
4. Khi nạp firmware, ưu tiên **USB 5 V** và ngắt tải motor nặng nếu upload lỗi.

---

## 2. Pin map (`config.h`)

### L298N → ESP32

| L298N | GPIO | Chức năng |
|-------|------|-----------|
| ENB | **32** | PWM motor **trái** (OUT1/OUT2) |
| IN1 | **27** | Hướng motor trái |
| IN2 | **26** | Hướng motor trái |
| ENA | **14** | PWM motor **phải** (OUT3/OUT4) |
| IN3 | **25** | Hướng motor phải |
| IN4 | **33** | Hướng motor phải |

### Motor + encoder

| Bên | Motor L298N | Encoder C1 | Encoder C2 |
|-----|-------------|------------|------------|
| Trái | OUT1, OUT2 | GPIO **18** | GPIO **19** |
| Phải | OUT3, OUT4 | GPIO **22** | GPIO **23** |

### Khác

| Peripheral | GPIO | Ghi chú |
|------------|------|---------|
| Buzzer | **5** | Output |
| Gas sensor | **36** | VP (SP) — analog input |

Firmware dùng `PWM_FREQ_HZ = 1000` (1 kHz — phù hợp L298N; TB6612 thường ~20 kHz) và `MAX_PWM_DUTY = 200`.

---

## 3. L298N wiring

| L298N pin | Connect to |
|-----------|------------|
| IN1 | ESP32 GPIO **27** |
| IN2 | ESP32 GPIO **26** |
| ENB | ESP32 GPIO **32** |
| OUT1, OUT2 | **Trái** JGB37 motor |
| IN3 | ESP32 GPIO **25** |
| IN4 | ESP32 GPIO **33** |
| ENA | ESP32 GPIO **14** |
| OUT3, OUT4 | **Phải** JGB37 motor |
| +5V / +12V | Nguồn logic/motor (tùy module) |
| GND | Common ground |

Bỏ jumper ENA/ENB trên module nếu đang dùng — firmware điều khiển PWM qua GPIO 14 và 32.

---

## 4. JGB37 encoder wiring

Mỗi motor JGB37 thường có 6 dây: M+, M−, VCC, GND, C1 (A), C2 (B).

### Encoder phải

| Dây | ESP32 |
|-----|-------|
| VCC | 3.3 V |
| GND | GND |
| C1 | GPIO **22** |
| C2 | GPIO **23** |

### Encoder trái

| Dây | ESP32 |
|-----|-------|
| VCC | 3.3 V |
| GND | GND |
| C1 | GPIO **18** |
| C2 | GPIO **19** |

Firmware dùng `INPUT_PULLUP` cho cả hai encoder. Ngắt trên **cạnh lên C1**; C2 xác định chiều quay.

---

## 5. ESP32 pins to avoid at boot

| GPIO | Risk |
|------|------|
| 0 | Boot mode |
| 2 | Boot / LED onboard |
| 12 | Flash voltage strap |
| 15 | Must be HIGH at boot |

---

## 6. Wiring checklist

- [ ] Nguồn chung GND giữa ESP32, L298N, encoder, pin
- [ ] Motor trái: OUT1/OUT2; encoder C1=18, C2=19
- [ ] Motor phải: OUT3/OUT4; encoder C1=22, C2=23
- [ ] L298N: ENB=32 (trái), ENA=14 (phải), IN1–IN4 đúng bảng trên
- [ ] Buzzer GPIO 5, gas sensor GPIO 36 (VP)
- [ ] `config.h` đã cập nhật và nạp lại firmware

---

## 7. Motor direction signs

Nếu bánh chạy ngược, sửa trong `config.h`:

```cpp
#define MOTOR_R_SIGN   1
#define MOTOR_L_SIGN  -1
#define ENC_R_SIGN     1
#define ENC_L_SIGN    -1
```

Chạy `motor_test/motor_test.ino` trước để hiệu chuẩn trim và encoder.
