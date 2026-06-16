# ESP32 Tank JGB37 — Wiring & Pin Configuration

Detailed wiring guide for this tank project using **ESP32 Dev Module**, **TB6612FNG** motor driver, and two **JGB37** gear motors with encoders.

Goals:
- Keep the existing pin mapping as much as possible.
- **Do not use GPIO12** — it causes boot/flash failures and memory issues on ESP32.

---

## 1. Power Supply (recommended)

Your intended power chain:

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
        └── TB6612FNG (VCC logic + VM motor supply)
```

### Recommended voltages

| Rail | Voltage | Notes |
|------|---------|-------|
| LM2596 output | **5.0 V** | ESP32 logic and TB6612 `VCC` |
| TB6612 `VM` | **6–12 V** | Motor supply; tune for your JGB37 motors |
| Encoder VCC | **3.3 V or 5 V** | Match encoder module rating |

### Critical rules

1. **Common ground** — tie GND from battery, XL4016, LM2596, ESP32, TB6612, and both encoders together.
2. **Decoupling capacitors**
   - 470–1000 µF near TB6612 `VM`
   - 100 µF + 0.1 µF near ESP32 power input
3. **Do not power ESP32 directly from 12 V** — always step down to 5 V first.
4. During flashing, prefer **USB 5 V** and disconnect heavy motor loads if upload fails.

---

## 2. Pin map (`config.h`)

### Current mapping (GPIO12 still present — avoid)

| Function | GPIO | Notes |
|----------|------|-------|
| Motor R PWM | 27 | TB6612 `PWMA` |
| Motor R IN1 | 26 | TB6612 `AIN1` |
| Motor R IN2 | 25 | TB6612 `AIN2` |
| Motor L PWM | 13 | TB6612 `PWMB` |
| Motor L IN1 | 33 | TB6612 `BIN1` |
| Motor L IN2 | 32 | TB6612 `BIN2` |
| Encoder R C1 | **12** | ⚠ Boot strap pin — **replace** |
| Encoder R C2 | 14 | Phase B |
| Encoder L C1 | 35 | Input-only |
| Encoder L C2 | 34 | Input-only |
| Buzzer | 15 | Active buzzer |
| Gas sensor | 36 | Analog input (input-only) |

### Recommended mapping (only one change)

Move the right encoder interrupt pin off GPIO12:

| Change | Old | New |
|--------|-----|-----|
| `PIN_ENC_R_C1` | 12 | **23** |

All other pins stay the same.

**Why GPIO23?**
- Safe for boot and flashing on ESP32 Dev Module.
- Works well as an encoder interrupt input.
- Minimal change from the original wiring.

### Quick fix in `config.h`

```cpp
// Before (causes flash/boot issues)
#define PIN_ENC_R_C1 12

// After (recommended)
#define PIN_ENC_R_C1 23
```

Also update `config.h.example` and `motor_test/motor_test.ino` if you use those files.

---

## 3. TB6612FNG wiring

| TB6612 pin | Connect to |
|------------|------------|
| `PWMA` | ESP32 GPIO **27** |
| `AIN1` | ESP32 GPIO **26** |
| `AIN2` | ESP32 GPIO **25** |
| `AO1`, `AO2` | Right JGB37 motor |
| `PWMB` | ESP32 GPIO **13** |
| `BIN1` | ESP32 GPIO **33** |
| `BIN2` | ESP32 GPIO **32** |
| `BO1`, `BO2` | Left JGB37 motor |
| `STBY` | **3.3 V** (via 10 kΩ pull-up, or direct 3.3 V if always enabled) |
| `VCC` | **3.3 V or 5 V** (match ESP32 logic level) |
| `VM` | Motor rail (6–12 V from XL4016/LM2596 chain) |
| `GND` | Common ground |

---

## 4. JGB37 encoder wiring

Each JGB37 motor typically has a 6-wire harness:
- Motor +: `M+`
- Motor −: `M−`
- Encoder VCC
- Encoder GND
- Encoder channel A (C1)
- Encoder channel B (C2)

### Right encoder

| Encoder wire | ESP32 |
|--------------|-------|
| VCC | 3.3 V (or 5 V if module requires it) |
| GND | GND |
| C1 (A) | GPIO **23** (was 12) |
| C2 (B) | GPIO **14** |

### Left encoder

| Encoder wire | ESP32 |
|--------------|-------|
| VCC | 3.3 V |
| GND | GND |
| C1 (A) | GPIO **35** |
| C2 (B) | GPIO **34** |

### Pull-up resistors

- **Right encoder (GPIO 14, 23):** firmware uses `INPUT_PULLUP` — external resistors optional.
- **Left encoder (GPIO 34, 35):** input-only pins have **no internal pull-up**. Add **10 kΩ pull-ups** from C1/C2 to 3.3 V (as noted in `odometry.cpp`).

Interrupts fire on **rising edge of C1**; C2 sets direction.

---

## 5. Other peripherals

| Peripheral | GPIO | Type |
|------------|------|------|
| Buzzer | 15 | Output |
| Gas sensor | 36 | Analog input (VP) |

---

## 6. ESP32 pins to avoid

These are **strap pins** — wrong levels at boot can block flashing or corrupt flash:

| GPIO | Risk |
|------|------|
| **12** | Must be LOW at boot (flash voltage select) — **do not use for encoder** |
| 0 | Boot mode |
| 2 | Boot / onboard LED on some boards |
| 15 | Must be HIGH at boot |

Prefer input-only pins (`34`, `35`, `36`, `39`) for sensors when possible.

### If upload fails

1. Disconnect external loads that may pull strap pins.
2. Hold **BOOT**, press **RESET**, release **BOOT**, then upload.
3. Use a good USB data cable and try lower upload speed (115200).
4. Power ESP32 from USB only while flashing.

---

## 7. Wiring checklist

- [ ] 3S battery → XL4016 → LM2596 → 5 V for ESP32 / TB6612 logic
- [ ] Motor VM on separate buck output (6–12 V)
- [ ] All grounds tied together
- [ ] Right encoder C1 moved from GPIO **12** to GPIO **23**
- [ ] 10 kΩ pull-ups on left encoder C1/C2 (GPIO 34, 35)
- [ ] TB6612 `STBY` tied high
- [ ] Decoupling caps on motor and ESP32 power rails
- [ ] `config.h` updated and re-flashed

---

## 8. Motor direction signs

Tune in `config.h` if a wheel runs backward:

```cpp
#define MOTOR_R_SIGN   1
#define MOTOR_L_SIGN  -1
#define ENC_R_SIGN     1
#define ENC_L_SIGN    -1
```
