/*
 * TEST MOTOR + ENCODER — ESP32 Tank (ban don gian)
 *
 * Triet ly: PWM 2 banh BANG NHAU + vong kin encoder tu giu thang.
 * KHONG can can bang PWM tay.
 *
 * Quy trinh 3 buoc:
 *   1. d  -> kiem chieu banh + encoder (sua sign neu sai)
 *   2. f  -> di thang, do quang duong, chinh counts/cm bang g/h
 *   3. r/l/b -> xoay 90 phai / 90 trai / 180, chinh turn counts bang [ ] { }
 *   p  -> in block config.h de copy, roi nap esp32-tank-jbg37.ino
 *
 * Board: ESP32 Dev Module | Serial 115200
 */

#if __has_include("../config.h")
#include "../config.h"
#else
#define PIN_MOTOR_L_PWM 32
#define PIN_MOTOR_L_IN1 27
#define PIN_MOTOR_L_IN2 26
#define PIN_MOTOR_R_PWM 14
#define PIN_MOTOR_R_IN1 25
#define PIN_MOTOR_R_IN2 33
#define PIN_ENC_R_C1 22
#define PIN_ENC_R_C2 23
#define PIN_ENC_L_C1 18
#define PIN_ENC_L_C2 19
#define PIN_BUZZER 5
#define MOTOR_R_SIGN  1
#define MOTOR_L_SIGN  1
#define ENC_R_SIGN    1
#define ENC_L_SIGN   -1
#define PWM_FREQ_HZ   1000
#define PWM_RES_BITS  8
#define MAX_PWM_DUTY  200
#define PWM_DUTY_LEFT   160
#define PWM_DUTY_RIGHT  160
#define TURN_PWM_LEFT   160
#define TURN_PWM_RIGHT  160
#define DEFAULT_COUNTS_PER_CM 111.0f
#define TURN_90_LEFT_COUNTS  1200L
#define TURN_90_RIGHT_COUNTS 1200L
#define TURN_180_COUNTS      2400L
#endif

// ---- Fallback cho cac tham so vong kin (neu config.h chua co) ----
#ifndef STRAIGHT_ENC_KP
#define STRAIGHT_ENC_KP 5
#endif
#ifndef STRAIGHT_DRIFT_TRIM
#define STRAIGHT_DRIFT_TRIM 0
#endif
#ifndef STRAIGHT_ENC_MAX_TRIM
#define STRAIGHT_ENC_MAX_TRIM 60
#endif
#ifndef STRAIGHT_DECEL_ZONE_PCT
#define STRAIGHT_DECEL_ZONE_PCT 12
#endif
#ifndef STRAIGHT_PWM_FLOOR
#define STRAIGHT_PWM_FLOOR 110
#endif
#ifndef PWM_MIN_DUTY
#define PWM_MIN_DUTY 70
#endif
#ifndef TURN_ENC_KP
#define TURN_ENC_KP 2
#endif
#ifndef TURN_ENC_MAX_TRIM
#define TURN_ENC_MAX_TRIM 28
#endif
#ifndef TURN_CRUISE_PCT
#define TURN_CRUISE_PCT 50
#endif
#ifndef TURN_MIN_RUN_MS
#define TURN_MIN_RUN_MS 280UL
#endif
#ifndef TURN_DECEL_ZONE_PCT
#define TURN_DECEL_ZONE_PCT 35
#endif
#ifndef TURN_PWM_MIN
#define TURN_PWM_MIN 80
#endif
#ifndef STRAIGHT_STEP_TIMEOUT_MS_PER_CM
#define STRAIGHT_STEP_TIMEOUT_MS_PER_CM 95
#define STRAIGHT_STEP_TIMEOUT_EXTRA_MS  2000
#endif

#define PWM_CH_RIGHT 0
#define PWM_CH_LEFT  1

#define TEST_DIST_CM 100   // quang duong test di thang

volatile long encCountR = 0;
volatile long encCountL = 0;

int motorRSign = MOTOR_R_SIGN;
int motorLSign = MOTOR_L_SIGN;
volatile int encRSign = ENC_R_SIGN;
volatile int encLSign = ENC_L_SIGN;

int drivePwm = PWM_DUTY_LEFT;          // PWM di thang (2 banh bang nhau)
int turnPwm  = TURN_PWM_LEFT;          // PWM xoay   (2 banh bang nhau)
float countsPerCm = DEFAULT_COUNTS_PER_CM;
long turn90Counts  = TURN_90_RIGHT_COUNTS;
long turn180Counts = TURN_180_COUNTS;
int  driftTrim     = STRAIGHT_DRIFT_TRIM;     // phan nghin: (+)=lai TRAI, (-)=lai PHAI
bool autoRunning = false;

// ---------- low level ----------
void IRAM_ATTR isrEncR() {
  if (digitalRead(PIN_ENC_R_C2)) encCountR += encRSign;
  else                           encCountR -= encRSign;
}
void IRAM_ATTR isrEncL() {
  if (digitalRead(PIN_ENC_L_C2)) encCountL += encLSign;
  else                           encCountL -= encLSign;
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
  digitalWrite(in1, forward ? HIGH : LOW);
  digitalWrite(in2, forward ? LOW : HIGH);
  writePwm(pwmPin, channel, abs(speed));
}
void setMotors(int left, int right) {
  driveMotor(PIN_MOTOR_L_IN1, PIN_MOTOR_L_IN2, PIN_MOTOR_L_PWM, PWM_CH_LEFT,  left  * motorLSign);
  driveMotor(PIN_MOTOR_R_IN1, PIN_MOTOR_R_IN2, PIN_MOTOR_R_PWM, PWM_CH_RIGHT, right * motorRSign);
}
void stopMotors() { setMotors(0, 0); }

void beep(int ms) {
  digitalWrite(PIN_BUZZER, HIGH); delay(ms); digitalWrite(PIN_BUZZER, LOW);
}
void resetEncoders() {
  noInterrupts(); encCountR = 0; encCountL = 0; interrupts();
}
void printEncoders() {
  Serial.printf("Enc PHAI=%ld  TRAI=%ld\n", encCountR, encCountL);
}
void flushSerial() { delay(20); while (Serial.available()) Serial.read(); }

bool abortPressed() {
  while (Serial.available()) if (Serial.read() == '0') return true;
  return false;
}

int clampDuty(int duty) {
  if (duty == 0) return 0;
  const int s = duty > 0 ? 1 : -1;
  return s * constrain(abs(duty), PWM_MIN_DUTY, MAX_PWM_DUTY);
}

// ---------- vong kin (giong het mission.cpp) ----------
void straightLoop(long startL, long startR, long encTarget) {
  const long dL = encCountL - startL;
  const long dR = encCountR - startR;
  const long progress = (max(0L, dL) + max(0L, dR)) / 2;
  const long err = (dL - dR) + (dR * driftTrim) / 1000;  // trim: bu lech co khi lien tuc

  int corr = constrain((int)(err * STRAIGHT_ENC_KP),
                       -STRAIGHT_ENC_MAX_TRIM, STRAIGHT_ENC_MAX_TRIM);

  int pwmL = drivePwm, pwmR = drivePwm;
  const long decelZone = max(1L, (encTarget * STRAIGHT_DECEL_ZONE_PCT) / 100);
  const long remaining = encTarget - progress;
  if (remaining > 0 && remaining < decelZone) {
    const int sc = (int)max(55L, remaining * 100 / decelZone);
    pwmL = max((int)STRAIGHT_PWM_FLOOR, pwmL * sc / 100);
    pwmR = max((int)STRAIGHT_PWM_FLOOR, pwmR * sc / 100);
  }
  setMotors(clampDuty(pwmL - corr), clampDuty(pwmR + corr));
}

int turnPwmScale(long progress, long encTarget, int basePwm) {
  const long cruiseEnd = (encTarget * TURN_CRUISE_PCT) / 100;
  const long decelZone = max(1L, (encTarget * TURN_DECEL_ZONE_PCT) / 100);
  const long remaining = encTarget - progress;
  if (progress < cruiseEnd) return basePwm;
  if (remaining <= 0 || remaining >= decelZone) return basePwm;
  const int sc = (int)max((long)TURN_PWM_MIN, remaining * 100 / decelZone);
  return max((int)TURN_PWM_MIN, basePwm * sc / 100);
}

void turnLoop(int sL, int sR, long startL, long startR, long encTarget) {
  const long dL = encCountL - startL;
  const long dR = encCountR - startR;
  const long progress = labs((dL - dR) / 2);
  const long magL = labs(dL), magR = labs(dR);

  int corr = constrain((int)((magL - magR) * TURN_ENC_KP),
                       -TURN_ENC_MAX_TRIM, TURN_ENC_MAX_TRIM);
  int pwmL = turnPwmScale(progress, encTarget, turnPwm);
  int pwmR = turnPwmScale(progress, encTarget, turnPwm);
  if (magL < magR) pwmL += corr;
  else if (magR < magL) pwmR += corr;
  setMotors(clampDuty(sL * pwmL), clampDuty(sR * pwmR));
}

// ---------- tests ----------
void runForwardCm(float cm) {
  if (autoRunning) { Serial.println("Dang chay — nhan 0 de dung"); return; }
  autoRunning = true;
  flushSerial();
  resetEncoders();

  const long encTarget = (long)(cm * countsPerCm);
  const unsigned long timeoutMs =
      (unsigned long)(cm * STRAIGHT_STEP_TIMEOUT_MS_PER_CM) + STRAIGHT_STEP_TIMEOUT_EXTRA_MS;

  Serial.printf("\n>> DI THANG %.0f cm (target=%ld counts, PWM=%d, Kp=%d)\n",
                cm, encTarget, drivePwm, STRAIGHT_ENC_KP);
  beep(50);

  const long startL = encCountL, startR = encCountR;
  unsigned long t0 = millis();
  long progress = 0, maxErr = 0;

  while (millis() - t0 < timeoutMs) {
    delay(20);
    straightLoop(startL, startR, encTarget);
    progress = (max(0L, encCountL - startL) + max(0L, encCountR - startR)) / 2;
    const long err = labs((encCountL - startL) - (encCountR - startR));
    if (err > maxErr) maxErr = err;
    if (progress >= encTarget) break;
    if (abortPressed()) { Serial.println("   Dung som (0)"); break; }
  }
  stopMotors();
  autoRunning = false;

  const long dL = encCountL - startL, dR = encCountR - startR;
  Serial.printf("   progress=%ld/%ld  dL=%ld dR=%ld  lech max|L-R|=%ld\n",
                progress, encTarget, dL, dR, maxErr);
  Serial.printf("   Uoc luong: %.1f cm\n", progress / countsPerCm);
  Serial.println("   -> Do quang duong THUC bang thuoc.");
  Serial.println("      Neu lech: counts/cm = progress / cm_thuc, chinh bang g/h.");
  if (maxErr > 120)
    Serial.println("   Bi nghieng nhieu -> tang STRAIGHT_ENC_KP (config.h).");
  else
    Serial.println("   2 banh bam sat -> di thang OK.");
}

void runTurn(int sL, int sR, const char* label, long encTarget) {
  if (autoRunning) { Serial.println("Dang chay — nhan 0 de dung"); return; }
  autoRunning = true;
  flushSerial();
  resetEncoders();

  Serial.printf("\n>> %s (target=%ld counts, PWM=%d)\n", label, encTarget, turnPwm);
  Serial.println("   (chi phim 0 moi dung som)");
  beep(50);

  const long startL = encCountL, startR = encCountR;
  unsigned long t0 = millis();
  long progress = 0;

  while (millis() - t0 < 12000UL) {
    delay(20);
    turnLoop(sL, sR, startL, startR, encTarget);
    progress = labs(((encCountL - startL) - (encCountR - startR)) / 2);
    if (millis() - t0 >= TURN_MIN_RUN_MS && progress >= encTarget) break;
    if (abortPressed()) { Serial.println("   Dung som (0)"); stopMotors(); autoRunning = false; return; }
  }
  stopMotors();
  autoRunning = false;

  Serial.printf("   progress=%ld/%ld  dL=%ld dR=%ld\n",
                progress, encTarget, encCountL - startL, encCountR - startR);
  Serial.println("   -> Do GOC thuc bang eke/giay.");
  Serial.println("      Thieu goc: tang counts ([ ] cho 90, { } cho 180).");
  Serial.println("      Du goc: giam counts.");
  beep(150);
}

void runDirectionCheck() {
  if (autoRunning) return;
  autoRunning = true;
  flushSerial();
  Serial.println("\n=== KIEM CHIEU BANH + ENCODER ===");
  Serial.printf("Sign: MOTOR_R=%d MOTOR_L=%d  ENC_R=%d ENC_L=%d\n",
                motorRSign, motorLSign, encRSign, encLSign);

  resetEncoders();
  Serial.println("\n>> PHAI tien 1.5s — banh phai phai quay TOI, enc PHAI phai TANG");
  setMotors(0, drivePwm); delay(1500); stopMotors();
  Serial.printf("   enc PHAI = %+ld  %s\n", encCountR,
                encCountR > 30 ? "OK" : "SAI -> u/j dao ENC, o/i dao MOTOR");

  delay(400); resetEncoders();
  Serial.println(">> TRAI tien 1.5s — banh trai phai quay TOI, enc TRAI phai TANG");
  setMotors(drivePwm, 0); delay(1500); stopMotors();
  Serial.printf("   enc TRAI = %+ld  %s\n", encCountL,
                encCountL > 30 ? "OK" : "SAI -> u/j dao ENC, o/i dao MOTOR");

  Serial.println("\nQuy tac: banh quay TOI nhung enc GIAM -> dao ENC sign (u/j).");
  Serial.println("         banh quay LUI (sai chieu) -> dao MOTOR sign (o/i).");
  autoRunning = false;
}

void flipMotor(bool left) {
  if (left) { motorLSign = -motorLSign; Serial.printf("MOTOR_L_SIGN=%d\n", motorLSign); }
  else      { motorRSign = -motorRSign; Serial.printf("MOTOR_R_SIGN=%d\n", motorRSign); }
}
void flipEnc(bool left) {
  if (left) { encLSign = -encLSign; Serial.printf("ENC_L_SIGN=%d\n", encLSign); }
  else      { encRSign = -encRSign; Serial.printf("ENC_R_SIGN=%d\n", encRSign); }
}

void printConfigBlock() {
  Serial.println("\n=== COPY VAO config.h ===");
  Serial.printf("#define MOTOR_R_SIGN           %d\n", motorRSign);
  Serial.printf("#define MOTOR_L_SIGN           %d\n", motorLSign);
  Serial.printf("#define ENC_R_SIGN             %d\n", encRSign);
  Serial.printf("#define ENC_L_SIGN             %d\n", encLSign);
  Serial.printf("#define PWM_DUTY_LEFT          %d\n", drivePwm);
  Serial.printf("#define PWM_DUTY_RIGHT         %d\n", drivePwm);
  Serial.printf("#define TURN_PWM_LEFT          %d\n", turnPwm);
  Serial.printf("#define TURN_PWM_RIGHT         %d\n", turnPwm);
  Serial.printf("#define STRAIGHT_ENC_KP        %d\n", STRAIGHT_ENC_KP);
  Serial.printf("#define STRAIGHT_DRIFT_TRIM    %d\n", driftTrim);
  Serial.printf("#define DEFAULT_COUNTS_PER_CM  %.1ff\n", countsPerCm);
  Serial.printf("#define TURN_90_LEFT_COUNTS    %ldL\n", turn90Counts);
  Serial.printf("#define TURN_90_RIGHT_COUNTS   %ldL\n", turn90Counts);
  Serial.printf("#define TURN_180_COUNTS        %ldL\n", turn180Counts);
  Serial.println("=========================");
  Serial.println("Sau do nap esp32-tank-jbg37.ino");
}

void printMenu() {
  Serial.println("\n=== MENU (don gian) ===");
  Serial.println("Buoc 1: d = kiem chieu banh + encoder");
  Serial.println("        o/i dao MOTOR_L/R sign | u/j dao ENC_L/R sign");
  Serial.println("Buoc 2: f = di thang 100cm  -> g/h chinh counts/cm");
  Serial.println("        . = lai TRAI hon | , = lai PHAI hon (xe van nghieng du enc deu)");
  Serial.println("Buoc 3: r = xoay PHAI 90 | l = xoay TRAI 90 | b = xoay 180");
  Serial.println("        [ ] chinh turn90 counts | { } chinh turn180 counts");
  Serial.println("Khac:   0 dung | e doc enc | x reset enc");
  Serial.println("        + - PWM di thang | < > PWM xoay | p in config | ? menu");
  Serial.printf("PWM thang=%d  PWM xoay=%d  counts/cm=%.1f  drift=%d  turn90=%ld  turn180=%ld\n",
                drivePwm, turnPwm, countsPerCm, driftTrim, turn90Counts, turn180Counts);
  Serial.println("=======================");
}

void handleCommand(char c) {
  switch (c) {
    case 'd': runDirectionCheck(); break;
    case 'f': runForwardCm(TEST_DIST_CM); break;
    // re phai that: trai LUI, phai TIEN | re trai that: trai TIEN, phai LUI
    case 'r': runTurn(-1, +1, "XOAY PHAI 90", turn90Counts); break;
    case 'l': runTurn(+1, -1, "XOAY TRAI 90", turn90Counts); break;
    case 'b': runTurn(-1, +1, "XOAY 180", turn180Counts); break;
    case '0': stopMotors(); Serial.println("DUNG"); break;
    case 'o': flipMotor(true); break;
    case 'i': flipMotor(false); break;
    case 'u': flipEnc(true); break;
    case 'j': flipEnc(false); break;
    case 'e': printEncoders(); break;
    case 'x': resetEncoders(); Serial.println("Reset enc=0"); break;
    case '+': case '=': drivePwm = constrain(drivePwm + 5, 80, MAX_PWM_DUTY);
              Serial.printf("PWM thang=%d\n", drivePwm); break;
    case '-': case '_': drivePwm = constrain(drivePwm - 5, 80, MAX_PWM_DUTY);
              Serial.printf("PWM thang=%d\n", drivePwm); break;
    case '<': turnPwm = constrain(turnPwm - 5, 80, MAX_PWM_DUTY);
              Serial.printf("PWM xoay=%d\n", turnPwm); break;
    case '>': turnPwm = constrain(turnPwm + 5, 80, MAX_PWM_DUTY);
              Serial.printf("PWM xoay=%d\n", turnPwm); break;
    case 'g': countsPerCm = constrain(countsPerCm - 1.0f, 40.0f, 160.0f);
              Serial.printf("counts/cm=%.1f\n", countsPerCm); break;
    case 'h': countsPerCm = constrain(countsPerCm + 1.0f, 40.0f, 160.0f);
              Serial.printf("counts/cm=%.1f\n", countsPerCm); break;
    case '.': driftTrim = constrain(driftTrim + 1, -30, 30);  // lai TRAI hon
              Serial.printf("drift trim=%d phan nghin (+ = lai TRAI)\n", driftTrim); break;
    case ',': driftTrim = constrain(driftTrim - 1, -30, 30);  // lai PHAI hon
              Serial.printf("drift trim=%d phan nghin (- = lai PHAI)\n", driftTrim); break;
    case '[': turn90Counts = constrain(turn90Counts - 25, 300, 3000);
              turn180Counts = turn90Counts * 2;
              Serial.printf("turn90=%ld turn180=%ld\n", turn90Counts, turn180Counts); break;
    case ']': turn90Counts = constrain(turn90Counts + 25, 300, 3000);
              turn180Counts = turn90Counts * 2;
              Serial.printf("turn90=%ld turn180=%ld\n", turn90Counts, turn180Counts); break;
    case '{': turn180Counts = constrain(turn180Counts - 25, 600, 6000);
              Serial.printf("turn180=%ld\n", turn180Counts); break;
    case '}': turn180Counts = constrain(turn180Counts + 25, 600, 6000);
              Serial.printf("turn180=%ld\n", turn180Counts); break;
    case 'p': printConfigBlock(); break;
    case '?': printMenu(); break;
    case '\r': case '\n': case ' ': break;
    default: Serial.printf("Phim '%c' khong dung — go ? xem menu\n", c); break;
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
  pinMode(PIN_ENC_L_C1, INPUT_PULLUP);
  pinMode(PIN_ENC_L_C2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_R_C1), isrEncR, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_L_C1), isrEncL, RISING);
  resetEncoders();

  beep(100);
  Serial.println("\n=== ESP32 TANK — MOTOR TEST (don gian) ===");
  Serial.println("PWM 2 banh bang nhau + vong kin encoder. Khong can can bang PWM.");
  printMenu();
}

void loop() {
  while (Serial.available()) {
    handleCommand(Serial.read());
  }
  static unsigned long lastPrint = 0;
  if (autoRunning && millis() - lastPrint >= 500) {
    lastPrint = millis();
    printEncoders();
  }
}
