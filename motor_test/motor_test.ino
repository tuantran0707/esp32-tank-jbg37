/*
 * TEST MOTOR + ENCODER — ESP32 Tank
 *
 * Chạy sketch này TRƯỚC khi nạp esp32-tank-jbg37.ino
 * Mục tiêu: tune trim thẳng, xoay 90°, encoder → copy vào config.h
 *
 * Board: ESP32 Dev Module | Serial 115200
 */

#if __has_include("../config.h")
#include "../config.h"
#else
#define PIN_MOTOR_R_PWM 27
#define PIN_MOTOR_R_IN1 26
#define PIN_MOTOR_R_IN2 25
#define PIN_MOTOR_L_PWM 13
#define PIN_MOTOR_L_IN1 33
#define PIN_MOTOR_L_IN2 32
#define PIN_ENC_R_C1 12
#define PIN_ENC_R_C2 14
#define PIN_ENC_L_C1 35
#define PIN_ENC_L_C2 34
#define PIN_BUZZER 15
#define MOTOR_R_SIGN  1
#define MOTOR_L_SIGN -1
#define ENC_R_SIGN    1
#define ENC_L_SIGN   -1
#define PWM_FREQ_HZ   20000
#define PWM_RES_BITS  8
#define MAX_PWM_DUTY  180
#define TURN_PWM_LEFT   165
#define TURN_PWM_RIGHT  120
#define DEFAULT_TURN_90_COUNTS 1650L
#endif

#ifndef DEFAULT_COUNTS_PER_CM
#define DEFAULT_COUNTS_PER_CM 103.0f
#endif

#ifndef STRAIGHT_STEP_TIMEOUT_MS_PER_CM
#define STRAIGHT_STEP_TIMEOUT_MS_PER_CM 95
#define STRAIGHT_STEP_TIMEOUT_EXTRA_MS  2000
#endif

#define TEST_PATH_CM 90

#define PWM_CH_RIGHT 0
#define PWM_CH_LEFT  1
#define STRAIGHT_TEST_MS 2000

volatile long encCountR = 0;
volatile long encCountL = 0;

int testPwm = MAX_PWM_DUTY;
int straightTrimFwd = 0;
int straightTrimRev = 0;
int turnPwmL = TURN_PWM_LEFT;
int turnPwmR = TURN_PWM_RIGHT;
long turn90Target = DEFAULT_TURN_90_COUNTS;
bool autoRunning = false;
float countsPerCm = DEFAULT_COUNTS_PER_CM;

// --- forward declarations ---
void setMotors(int left, int right);
void stopMotors();
void beep(int ms);
void flushSerial();
void printEncoders();
void resetEncoders();
void printMenu();
void printConfigBlock();
void runForwardCm(float cm);

long encAvg() {
  return (encCountR + encCountL) / 2;
}

long turnProgress(long startL, long startR) {
  const long dL = encCountL - startL;
  const long dR = encCountR - startR;
  return (dL - dR) / 2;
}

void applyStraightTrim(int& left, int& right) {
  if (left > 0 && right > 0 && left == right) {
    right = constrain(right - straightTrimFwd, 0, 255);
  } else if (left < 0 && right < 0 && left == right) {
    left = constrain(left + straightTrimRev, -255, 0);
  }
}

void straightDuties(int basePwm, int& left, int& right) {
  left = basePwm;
  right = basePwm;
  applyStraightTrim(left, right);
}

bool runStepMs(int left, int right, int durationMs) {
  setMotors(left, right);
  if (left != 0 || right != 0) beep(50);
  const int ticks = durationMs / 100;
  for (int t = 0; t < ticks; t++) {
    delay(100);
    if (Serial.available()) {
      flushSerial();
      stopMotors();
      return false;
    }
  }
  stopMotors();
  return true;
}

bool runStepMsBalanced(int basePwm, int durationMs) {
  int left, right;
  straightDuties(basePwm, left, right);
  Serial.printf("   PWM: L=%d R=%d (trimF=%d trimR=%d)\n",
                left, right, straightTrimFwd, straightTrimRev);
  return runStepMs(left, right, durationMs);
}

bool runStraightTest(int basePwm, const char* label) {
  flushSerial();
  resetEncoders();
  Serial.printf("\n>> %s\n", label);

  int left, right;
  straightDuties(basePwm, left, right);
  Serial.printf("   PWM: L=%d R=%d\n", left, right);

  if (!runStepMs(left, right, STRAIGHT_TEST_MS)) {
    Serial.println("   (dung som)");
    return false;
  }

  const long dR = encCountR;
  const long dL = encCountL;
  const long diff = dL - dR;
  Serial.printf("   Delta 2s — PHAI: %ld  TRAI: %ld  (L-R=%ld)\n", dR, dL, diff);
  if (basePwm > 0) {
    if (diff > 20) Serial.println("   Goi y: lech TRAI → tang trimF (phim ))");
    else if (diff < -20) Serial.println("   Goi y: lech PHAI → giam trimF (phim ()");
    else Serial.println("   Goi y: tien gan thang, co the copy config (p)");
  } else if (basePwm < 0) {
    if (diff > 20) Serial.println("   Goi y: lech khi lui → tang trimR (phim x)");
    else if (diff < -20) Serial.println("   Goi y: lech khi lui → giam trimR (phim z)");
    else Serial.println("   Goi y: lui gan thang, co the copy config (p)");
  }
  return true;
}

long forwardProgress(long startR, long startL) {
  const long dR = encCountR - startR;
  const long dL = encCountL - startL;
  if (dR <= 0 || dL <= 0) return 0;
  return (dR + dL) / 2;
}

void runForwardCm(float cm) {
  if (autoRunning) {
    Serial.println("Dang chay test khac — nhan 0 de dung truoc");
    return;
  }
  autoRunning = true;
  resetEncoders();

  const long encTarget = (long)(cm * countsPerCm);
  const unsigned long timeoutMs = (unsigned long)(cm * STRAIGHT_STEP_TIMEOUT_MS_PER_CM)
                                  + STRAIGHT_STEP_TIMEOUT_EXTRA_MS;

  int left, right;
  straightDuties(testPwm, left, right);

  Serial.printf("\n>> TIEN %.0f cm (target=%ld counts, timeout=%lums)\n",
                cm, encTarget, timeoutMs);
  Serial.printf("   PWM: L=%d R=%d (trimF=%d trimR=%d)\n",
                left, right, straightTrimFwd, straightTrimRev);
  Serial.printf("   Vi tri logic: (0,0) -> (0,%.0f)\n", cm);

  setMotors(left, right);
  beep(50);

  const long startR = encCountR;
  const long startL = encCountL;
  unsigned long t0 = millis();
  long progress = 0;

  while (millis() - t0 < timeoutMs) {
    delay(20);
    progress = forwardProgress(startR, startL);
    if (progress >= encTarget) {
      Serial.printf("   Dat target sau %lu ms, progress=%ld\n",
                    millis() - t0, progress);
      break;
    }
    if (Serial.available()) {
      const char c = Serial.read();
      if (c == '0') {
        while (Serial.available()) Serial.read();
        stopMotors();
        autoRunning = false;
        Serial.println("   Dung som");
        return;
      }
    }
  }
  stopMotors();
  autoRunning = false;

  const long dR = encCountR - startR;
  const long dL = encCountL - startL;
  Serial.printf("   Ket thuc: progress=%ld target=%ld (dR=%ld dL=%ld)\n",
                progress, encTarget, dR, dL);
  Serial.printf("   Uoc luong quang duong: %.1f cm (counts/cm=%.1f)\n",
                progress / countsPerCm, countsPerCm);
  printEncoders();
  if (progress < encTarget * 9 / 10) {
    Serial.println("   CANH BAO: chua du quang duong — thu phim 5 (CA 2 tien)");
  } else {
    beep(200);
    Serial.println("   OK");
  }
}

bool runTurnStep(int left, int right, const char* label) {
  const long startL = encCountL;
  const long startR = encCountR;
  Serial.printf(">> XOAY: %s  (L=%d R=%d, target=%ld)\n",
                label, left, right, turn90Target);
  setMotors(left, right);
  beep(50);

  unsigned long t0 = millis();
  long reached = 0;
  while (millis() - t0 < 8000UL) {
    delay(20);
    reached = turnProgress(startL, startR);
    if (labs(reached) >= turn90Target) {
      Serial.printf("   Dat target sau %lu ms, progress=%ld\n", millis() - t0, reached);
      break;
    }
    if (Serial.available()) {
      flushSerial();
      stopMotors();
      return false;
    }
  }
  stopMotors();
  Serial.printf("   progress=%ld (target=%ld)\n", reached, turn90Target);
  return true;
}

void IRAM_ATTR isrEncR() {
  if (digitalRead(PIN_ENC_R_C2)) encCountR += ENC_R_SIGN;
  else                           encCountR -= ENC_R_SIGN;
}

void IRAM_ATTR isrEncL() {
  if (digitalRead(PIN_ENC_L_C2)) encCountL += ENC_L_SIGN;
  else                           encCountL -= ENC_L_SIGN;
}

void initPwmPin(uint8_t pin, uint8_t channel) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  ledcAttach(pin, PWM_FREQ_HZ, PWM_RES_BITS);
#else
  ledcSetup(channel, PWM_FREQ_HZ, PWM_RES_BITS);
  ledcAttachPin(pin, channel);
#endif
}

void writePwm(uint8_t pin, uint8_t channel, uint8_t duty) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  ledcWrite(pin, duty);
#else
  ledcWrite(channel, duty);
#endif
}

void driveMotor(int in1, int in2, uint8_t pwmPin, uint8_t channel, int speed) {
  speed = constrain(speed, -255, 255);
  const bool forward = speed >= 0;
  const int pwm = abs(speed);
  digitalWrite(in1, forward ? HIGH : LOW);
  digitalWrite(in2, forward ? LOW : HIGH);
  writePwm(pwmPin, channel, pwm);
}

void setMotors(int left, int right) {
  driveMotor(PIN_MOTOR_L_IN1, PIN_MOTOR_L_IN2, PIN_MOTOR_L_PWM, PWM_CH_LEFT,
             left * MOTOR_L_SIGN);
  driveMotor(PIN_MOTOR_R_IN1, PIN_MOTOR_R_IN2, PIN_MOTOR_R_PWM, PWM_CH_RIGHT,
             right * MOTOR_R_SIGN);
}

void setMotorsBalanced(int basePwm) {
  int left, right;
  straightDuties(basePwm, left, right);
  Serial.printf("PWM: L=%d R=%d (trimF=%d trimR=%d)\n",
                left, right, straightTrimFwd, straightTrimRev);
  setMotors(left, right);
}

void stopMotors() {
  setMotors(0, 0);
}

void beep(int ms) {
  digitalWrite(PIN_BUZZER, HIGH);
  delay(ms);
  digitalWrite(PIN_BUZZER, LOW);
}

void flushSerial() {
  delay(20);
  while (Serial.available()) Serial.read();
}

void printEncoders() {
  Serial.printf("Enc PHAI: %ld  |  Enc TRAI: %ld  |  TB: %ld\n",
                encCountR, encCountL, encAvg());
}

void resetEncoders() {
  noInterrupts();
  encCountR = 0;
  encCountL = 0;
  interrupts();
}

void printConfigBlock() {
  Serial.println();
  Serial.println("=== COPY VAO config.h ===");
  Serial.printf("#define MAX_PWM_DUTY             %d\n", MAX_PWM_DUTY);
  Serial.printf("#define TURN_PWM_LEFT            %d\n", turnPwmL);
  Serial.printf("#define TURN_PWM_RIGHT           %d\n", turnPwmR);
  Serial.printf("#define STRAIGHT_FWD_RIGHT_TRIM  %d\n", straightTrimFwd);
  Serial.printf("#define STRAIGHT_REV_LEFT_TRIM   %d\n", straightTrimRev);
  Serial.printf("#define DEFAULT_COUNTS_PER_CM  %.1ff\n", countsPerCm);
  Serial.printf("#define DEFAULT_TURN_90_COUNTS   %ldL\n", turn90Target);
  Serial.println("=========================");
  Serial.println("Sau do nạp lai esp32-tank-jbg37.ino");
}

void printMenu() {
  Serial.println();
  Serial.println("=== MOTOR TEST (truoc firmware chinh) ===");
  Serial.println("1/2  PHAI  tien/lui");
  Serial.println("3/4  TRAI  tien/lui");
  Serial.println("5/6  CA 2  tien/lui (co trim)");
  Serial.println("7/8  xoay phai/trai (giu phim)");
  Serial.println("f/b  test thang 2s tien/lui + delta encoder");
  Serial.println("t/y  xoay 90 do (dung encoder)");
  Serial.println("a    test day du tu dong");
  Serial.println("m    (0,0)->(0,90) tien 90cm bang encoder + trim");
  Serial.println("c    calibrate: tien→lui→xoay→in config");
  Serial.println("p    in block config.h de copy");
  Serial.println("0    DUNG  |  e encoder  |  r reset enc");
  Serial.println("+/-  PWM tien/lui (mac dinh = MAX_PWM_DUTY)");
  Serial.println("(/)  trimF tien -/+1   z/x  trimR lui -/+1  (mac dinh 0/0)");
  Serial.println("g/h  counts/cm -/+1  (dung thuoc met: counts/cm = progress / cm thuc te)");
  Serial.println("[/]  PWM xoay L   {/} PWM xoay R   9/, turn90");
  Serial.printf("PWM=%d trimF=%d trimR=%d turnL=%d turnR=%d turn90=%ld\n",
                testPwm, straightTrimFwd, straightTrimRev,
                turnPwmL, turnPwmR, turn90Target);
  Serial.printf("counts/cm=%.1f  path m=%dcm\n", countsPerCm, TEST_PATH_CM);
  Serial.println("=========================================");
}

void runCalibrationSuite() {
  flushSerial();
  autoRunning = true;
  Serial.println("\n>> CALIBRATE — tune (/) trimF, z/x trimR, roi chay lai c\n");

  if (!runStraightTest(testPwm, "BUOC 1/4: CA 2 TIEN (co trim)")) goto done;
  if (!runStepMs(0, 0, 500)) goto done;

  if (!runStraightTest(-testPwm, "BUOC 2/4: CA 2 LUI (co trim)")) goto done;
  if (!runStepMs(0, 0, 500)) goto done;

  resetEncoders();
  if (!runTurnStep(turnPwmL, -turnPwmR, "BUOC 3/4: XOAY PHAI 90")) goto done;
  if (!runStepMs(0, 0, 500)) goto done;

  resetEncoders();
  if (!runTurnStep(-turnPwmL, turnPwmR, "BUOC 4/4: XOAY TRAI 90")) goto done;

  beep(200);
  Serial.println("\n>> CALIBRATE xong.");
  printConfigBlock();

done:
  autoRunning = false;
}

void runAutoTest() {
  flushSerial();
  autoRunning = true;
  Serial.println("\n>> AUTO TEST (nhan phim bat ky de dung)");

  if (!runStepMsBalanced(testPwm, STRAIGHT_TEST_MS)) goto aborted;
  Serial.println(">> CA 2 TIEN"); printEncoders();

  if (!runStepMs(0, 0, 500)) goto aborted;

  if (!runStepMsBalanced(-testPwm, STRAIGHT_TEST_MS)) goto aborted;
  Serial.println(">> CA 2 LUI"); printEncoders();

  if (!runStepMs(0, 0, 500)) goto aborted;

  if (!runStepMs(0, testPwm, STRAIGHT_TEST_MS)) goto aborted;
  Serial.println(">> PHAI TIEN"); printEncoders();

  if (!runStepMs(0, 0, 500)) goto aborted;

  if (!runStepMs(testPwm, 0, STRAIGHT_TEST_MS)) goto aborted;
  Serial.println(">> TRAI TIEN"); printEncoders();

  if (!runStepMs(0, 0, 500)) goto aborted;

  resetEncoders();
  if (!runTurnStep(turnPwmL, -turnPwmR, "XOAY PHAI 90")) goto aborted;
  if (!runStepMs(0, 0, 500)) goto aborted;

  resetEncoders();
  if (!runTurnStep(-turnPwmL, turnPwmR, "XOAY TRAI 90")) goto aborted;

  stopMotors();
  autoRunning = false;
  beep(200);
  Serial.println(">> AUTO TEST xong.");
  printConfigBlock();
  return;

aborted:
  autoRunning = false;
  Serial.println(">> AUTO TEST dung som");
}

void handleCommand(char c) {
  if (c == '\r' || c == '\n' || c == ' ') return;

  switch (c) {
    case '1': setMotors(0, testPwm);  Serial.println("PHAI tien"); break;
    case '2': setMotors(0, -testPwm); Serial.println("PHAI lui");  break;
    case '3': setMotors(testPwm, 0);  Serial.println("TRAI tien"); break;
    case '4': setMotors(-testPwm, 0); Serial.println("TRAI lui");  break;
    case '5': setMotorsBalanced(testPwm);  Serial.println("CA 2 tien"); break;
    case '6': setMotorsBalanced(-testPwm); Serial.println("CA 2 lui");  break;
    case '7': setMotors(turnPwmL, -turnPwmR); Serial.println("Xoay PHAI"); break;
    case '8': setMotors(-turnPwmL, turnPwmR); Serial.println("Xoay TRAI"); break;
    case 'f': if (!autoRunning) runStraightTest(testPwm, "TEST THANG TIEN 2s"); break;
    case 'b': if (!autoRunning) runStraightTest(-testPwm, "TEST THANG LUI 2s"); break;
    case '0': stopMotors(); Serial.println("DUNG"); break;
    case 'e': printEncoders(); break;
    case 'r': resetEncoders(); Serial.println("Reset encoder = 0"); break;
    case 't': if (!autoRunning) { resetEncoders(); runTurnStep(turnPwmL, -turnPwmR, "PHAI 90"); } break;
    case 'y': if (!autoRunning) { resetEncoders(); runTurnStep(-turnPwmL, turnPwmR, "TRAI 90"); } break;
    case 'p': printConfigBlock(); break;
    case 'c': if (!autoRunning) runCalibrationSuite(); break;
    case '+':
    case '=': testPwm = constrain(testPwm + 10, 30, 255); Serial.printf("PWM=%d\n", testPwm); break;
    case '-':
    case '_': testPwm = constrain(testPwm - 10, 30, 255); Serial.printf("PWM=%d\n", testPwm); break;
    case '(': straightTrimFwd = constrain(straightTrimFwd - 1, 0, 40);
              Serial.printf("trimF=%d\n", straightTrimFwd); break;
    case ')': straightTrimFwd = constrain(straightTrimFwd + 1, 0, 40);
              Serial.printf("trimF=%d\n", straightTrimFwd); break;
    case '{': turnPwmR = constrain(turnPwmR - 5, 80, 255); Serial.printf("turnR=%d\n", turnPwmR); break;
    case '}': turnPwmR = constrain(turnPwmR + 5, 80, 255); Serial.printf("turnR=%d\n", turnPwmR); break;
    case 'z': straightTrimRev = constrain(straightTrimRev - 1, 0, 40);
              Serial.printf("trimR=%d\n", straightTrimRev); break;
    case 'x': straightTrimRev = constrain(straightTrimRev + 1, 0, 40);
              Serial.printf("trimR=%d\n", straightTrimRev); break;
    case '[': turnPwmL = constrain(turnPwmL - 5, 80, 255); Serial.printf("turnL=%d\n", turnPwmL); break;
    case ']': turnPwmL = constrain(turnPwmL + 5, 80, 255); Serial.printf("turnL=%d\n", turnPwmL); break;
    case '9': turn90Target = constrain(turn90Target + 50, 500, 4000);
              Serial.printf("turn90=%ld\n", turn90Target); break;
    case ',': turn90Target = constrain(turn90Target - 50, 500, 4000);
              Serial.printf("turn90=%ld\n", turn90Target); break;
    case 'g': countsPerCm = constrain(countsPerCm - 1.0f, 40.0f, 120.0f);
              Serial.printf("counts/cm=%.1f\n", countsPerCm); break;
    case 'h': countsPerCm = constrain(countsPerCm + 1.0f, 40.0f, 120.0f);
              Serial.printf("counts/cm=%.1f\n", countsPerCm); break;
    case 'a': if (!autoRunning) runAutoTest(); break;
    case 'm': runForwardCm(TEST_PATH_CM); break;
    case '?': printMenu(); break;
    default:
      Serial.printf("Phim '%c' khong ho tro (go ? xem menu)\n", c);
      break;
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);
  
  pinMode(PIN_MOTOR_R_IN1, OUTPUT);
  pinMode(PIN_MOTOR_R_IN2, OUTPUT);
  pinMode(PIN_MOTOR_L_IN1, OUTPUT);
  pinMode(PIN_MOTOR_L_IN2, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_BUZZER, LOW);

  initPwmPin(PIN_MOTOR_R_PWM, PWM_CH_RIGHT);
  initPwmPin(PIN_MOTOR_L_PWM, PWM_CH_LEFT);
  stopMotors();

  pinMode(PIN_ENC_R_C1, INPUT_PULLUP);
  pinMode(PIN_ENC_R_C2, INPUT_PULLUP);
  pinMode(PIN_ENC_L_C1, INPUT);
  pinMode(PIN_ENC_L_C2, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_R_C1), isrEncR, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_L_C1), isrEncL, RISING);
  resetEncoders();

  beep(100);
  Serial.println("\n=== ESP32 TANK — MOTOR TEST ===");
  Serial.println("Chay sketch nay truoc esp32-tank-jbg37.ino");
  Serial.println("Quy trinh: f/b + (/) z/x tune trim → p copy config.h → nạp firmware chinh");
  Serial.println("[v2] phim m = test tien 90cm thang (0,0)->(0,90)");
  printMenu();
}

void loop() {
  if (Serial.available()) handleCommand(Serial.read());

  static unsigned long lastPrint = 0;
  if (autoRunning && millis() - lastPrint >= 500) {
    lastPrint = millis();
    printEncoders();
  }
}
