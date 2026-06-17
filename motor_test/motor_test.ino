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
#define PWM_DUTY_LEFT   200
#define PWM_DUTY_RIGHT  160
#define TURN_PWM_LEFT   194
#define TURN_PWM_RIGHT  141
#define DEFAULT_TURN_90_COUNTS 601L
#endif

#ifndef DEFAULT_COUNTS_PER_CM
#define DEFAULT_COUNTS_PER_CM 111.0f
#endif

#ifndef STRAIGHT_STEP_TIMEOUT_MS_PER_CM
#define STRAIGHT_STEP_TIMEOUT_MS_PER_CM 95
#define STRAIGHT_STEP_TIMEOUT_EXTRA_MS  2000
#endif

#ifndef PWM_DUTY_LEFT
#define PWM_DUTY_LEFT MAX_PWM_DUTY
#endif
#ifndef PWM_DUTY_RIGHT
#define PWM_DUTY_RIGHT MAX_PWM_DUTY
#endif

#define TEST_PATH_CM 90

#define PWM_CH_RIGHT 0
#define PWM_CH_LEFT  1
#define STRAIGHT_TEST_MS 2000

#define DIRECTION_TEST_MS 2000

volatile long encCountR = 0;
volatile long encCountL = 0;

int motorRSign = MOTOR_R_SIGN;
int motorLSign = MOTOR_L_SIGN;
volatile int encRSign = ENC_R_SIGN;
volatile int encLSign = ENC_L_SIGN;

int testPwmL = PWM_DUTY_LEFT;
int testPwmR = PWM_DUTY_RIGHT;
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
void runDirectionTest();
void runBalanceTest(bool autoApply);
void runBasicDriveTest();
void cmdStop();
void cmdForwardTest();
void cmdBackwardTest();
void cmdTurnRight90();
void cmdTurnLeft90();
void handleLineCommand(const String& raw);
void flipMotorSign(bool left);
void flipEncSign(bool left);

long encAvg() {
  return (encCountR + encCountL) / 2;
}

long turnProgress(long startL, long startR) {
  const long dL = encCountL - startL;
  const long dR = encCountR - startR;
  return (dL - dR) / 2;
}

void applyStraightTrim(int& left, int& right) {
  if (left > 0 && right > 0) {
    right = constrain(right - straightTrimFwd, 0, MAX_PWM_DUTY);
  } else if (left < 0 && right < 0) {
    left = constrain(left + straightTrimRev, -MAX_PWM_DUTY, 0);
  }
}

void straightDuties(int baseSign, int& left, int& right) {
  if (baseSign >= 0) {
    left = testPwmL;
    right = testPwmR;
  } else {
    left = -testPwmL;
    right = -testPwmR;
  }
  applyStraightTrim(left, right);
}

void adjustBothPwm(int delta) {
  testPwmL = constrain(testPwmL + delta, 30, MAX_PWM_DUTY);
  testPwmR = constrain(testPwmR + delta, 30, MAX_PWM_DUTY);
  Serial.printf("PWM TRAI=%d PHAI=%d\n", testPwmL, testPwmR);
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

bool runStepMsBalanced(int baseSign, int durationMs) {
  int left, right;
  straightDuties(baseSign, left, right);
  Serial.printf("   PWM: L=%d R=%d (trimF=%d trimR=%d)\n",
                left, right, straightTrimFwd, straightTrimRev);
  return runStepMs(left, right, durationMs);
}

bool runStraightTest(int baseSign, const char* label) {
  flushSerial();
  resetEncoders();
  Serial.printf("\n>> %s\n", label);

  int left, right;
  straightDuties(baseSign, left, right);
  Serial.printf("   PWM: L=%d R=%d\n", left, right);

  if (!runStepMs(left, right, STRAIGHT_TEST_MS)) {
    Serial.println("   (dung som)");
    return false;
  }

  const long dR = encCountR;
  const long dL = encCountL;
  const long diff = dL - dR;
  Serial.printf("   Delta 2s — PHAI: %ld  TRAI: %ld  (L-R=%ld)\n", dR, dL, diff);
  if (baseSign > 0) {
    if (dR > 50 && dL < -50)
      Serial.println("   CANH BAO: TRAI chay NGUOC → phim o (dao MOTOR_L_SIGN)");
    else if (dL > 50 && dR < -50)
      Serial.println("   CANH BAO: PHAI chay NGUOC → phim i (dao MOTOR_R_SIGN)");
    else if (diff > 20) Serial.println("   Goi y: TRAI nhanh hon → giam PWM trai (;) hoac tang phai (/)");
    else if (diff < -20) Serial.println("   Goi y: PHAI nhanh hon → giam PWM phai (.) hoac tang trai (')");
    else Serial.println("   Goi y: tien gan deu, co the copy config (p)");
  } else if (baseSign < 0) {
    if (diff > 20) Serial.println("   Goi y: lech khi lui → tang trimR (phim x)");
    else if (diff < -20) Serial.println("   Goi y: lech khi lui → giam trimR (phim z)");
    else Serial.println("   Goi y: lui gan deu, co the copy config (p)");
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
  straightDuties(1, left, right);

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
  const int pwm = abs(speed);
  digitalWrite(in1, forward ? HIGH : LOW);
  digitalWrite(in2, forward ? LOW : HIGH);
  writePwm(pwmPin, channel, pwm);
}

void setMotors(int left, int right) {
  driveMotor(PIN_MOTOR_L_IN1, PIN_MOTOR_L_IN2, PIN_MOTOR_L_PWM, PWM_CH_LEFT,
             left * motorLSign);
  driveMotor(PIN_MOTOR_R_IN1, PIN_MOTOR_R_IN2, PIN_MOTOR_R_PWM, PWM_CH_RIGHT,
             right * motorRSign);
}

void setMotorsBalanced(int baseSign) {
  int left, right;
  straightDuties(baseSign, left, right);
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
  Serial.printf("#define MOTOR_R_SIGN             %d\n", motorRSign);
  Serial.printf("#define MOTOR_L_SIGN             %d\n", motorLSign);
  Serial.printf("#define ENC_R_SIGN               %d\n", encRSign);
  Serial.printf("#define ENC_L_SIGN               %d\n", encLSign);
  Serial.printf("#define PWM_DUTY_LEFT            %d\n", testPwmL);
  Serial.printf("#define PWM_DUTY_RIGHT           %d\n", testPwmR);
  Serial.printf("#define MAX_PWM_DUTY             %d\n", MAX_PWM_DUTY);
  Serial.printf("#define TURN_PWM_LEFT            %d\n", turnPwmL);
  Serial.printf("#define TURN_PWM_RIGHT           %d\n", turnPwmR);
  Serial.printf("#define STRAIGHT_FWD_RIGHT_TRIM  %d\n",
                constrain(testPwmL - testPwmR + straightTrimFwd, 0, 80));
  Serial.printf("#define STRAIGHT_REV_LEFT_TRIM   %d\n",
                constrain(testPwmR - testPwmL + straightTrimRev, 0, 80));
  Serial.printf("#define DEFAULT_COUNTS_PER_CM  %.1ff\n", countsPerCm);
  Serial.printf("#define DEFAULT_TURN_90_COUNTS   %ldL\n", turn90Target);
  Serial.println("=========================");
  Serial.println("Sau do nạp lai esp32-tank-jbg37.ino");
}

void flipMotorSign(bool left) {
  if (left) {
    motorLSign = -motorLSign;
    Serial.printf("MOTOR_L_SIGN = %d\n", motorLSign);
  } else {
    motorRSign = -motorRSign;
    Serial.printf("MOTOR_R_SIGN = %d\n", motorRSign);
  }
}

void flipEncSign(bool left) {
  if (left) {
    encLSign = -encLSign;
    Serial.printf("ENC_L_SIGN = %d\n", encLSign);
  } else {
    encRSign = -encRSign;
    Serial.printf("ENC_R_SIGN = %d\n", encRSign);
  }
}

const char* directionVerdict(long delta) {
  if (delta > 20)
    return "enc TANG — neu banh chay TIEN that → OK";
  if (delta < -20)
    return "enc GIAM — banh chay LUI? phim o/i | banh chay TIEN? phim u/j";
  return "MO HO — thu tang PWM (+) hoac kiem day encoder";
}

void runWheelDirectionTest(const char* label, int leftDuty, int rightDuty) {
  resetEncoders();
  Serial.printf("\n>> %s (PWM L=%d R=%d, 2s)\n", label, leftDuty, rightDuty);
  runStepMs(leftDuty, rightDuty, DIRECTION_TEST_MS);
  const long dR = encCountR;
  const long dL = encCountL;
  if (rightDuty != 0) {
    Serial.printf("   PHAI: enc delta = %+ld → %s\n", dR, directionVerdict(dR));
  }
  if (leftDuty != 0) {
    Serial.printf("   TRAI: enc delta = %+ld → %s\n", dL, directionVerdict(dL));
  }
}

void runBalanceTest(bool autoApply) {
  if (autoRunning) return;
  autoRunning = true;
  flushSerial();

  Serial.println("\n=== CAN BANG PWM 2 BANH ===");
  Serial.printf("PWM hien tai: TRAI=%d PHAI=%d\n", testPwmL, testPwmR);

  resetEncoders();
  Serial.printf("\n>> Buoc 1/3: chi TRAI tien 2s @ PWM %d\n", testPwmL);
  runStepMs(testPwmL, 0, DIRECTION_TEST_MS);
  const long encL = labs(encCountL);

  delay(400);
  resetEncoders();
  Serial.printf(">> Buoc 2/3: chi PHAI tien 2s @ PWM %d\n", testPwmR);
  runStepMs(0, testPwmR, DIRECTION_TEST_MS);
  const long encR = labs(encCountR);

  Serial.printf("\n   Encoder 2s — TRAI: %ld  PHAI: %ld\n", encL, encR);

  if (encL < 20 || encR < 20) {
    Serial.println("   LOI: encoder qua thap — kiem chieu banh (phim d), day, PWM (+)");
    autoRunning = false;
    return;
  }

  const int oldL = testPwmL;
  const int oldR = testPwmR;
  const int avgPwm = (testPwmL + testPwmR) / 2;
  int newL = constrain((int)((long)avgPwm * encR / encL), 30, MAX_PWM_DUTY);
  int newR = constrain((int)((long)avgPwm * encL / encR), 30, MAX_PWM_DUTY);

  Serial.printf("   Goi y PWM can bang (giu TB ~%d): TRAI %d→%d  PHAI %d→%d\n",
                avgPwm, oldL, newL, oldR, newR);

  if (autoApply) {
    testPwmL = newL;
    testPwmR = newR;
    Serial.println("   >> Da ap dung PWM moi");

    delay(400);
    resetEncoders();
    Serial.println(">> Buoc 3/3: CA 2 tien 2s voi PWM da can bang");
    int left, right;
    straightDuties(1, left, right);
    Serial.printf("   PWM: L=%d R=%d\n", left, right);
    runStepMs(left, right, DIRECTION_TEST_MS);
    const long dL = encCountL;
    const long dR = encCountR;
    const long diff = dL - dR;
    Serial.printf("   Ket qua: PHAI=%ld TRAI=%ld (L-R=%+ld)\n", dR, dL, diff);
    if (labs(diff) <= 20) {
      Serial.println("   OK — 2 banh gan deu, tiep tuc test chieu (d) hoac calibrate (c)");
      beep(200);
    } else if (diff > 20) {
      Serial.println("   Van lech TRAI — giam PWM trai (;) hoac chay lai *");
    } else {
      Serial.println("   Van lech PHAI — giam PWM phai (.) hoac chay lai *");
    }
  } else {
    Serial.println("   >> Nhan * de ap dung PWM goi y va xac nhan buoc 3");
  }

  autoRunning = false;
}

void cmdStop() {
  stopMotors();
  Serial.printf("DUNG | enc PHAI=%ld TRAI=%ld\n", encCountR, encCountL);
}

void cmdForwardTest() {
  if (autoRunning) return;
  runStraightTest(1, "TIEN THANG 2s");
}

void cmdBackwardTest() {
  if (autoRunning) return;
  runStraightTest(-1, "LUI THANG 2s");
}

void cmdTurnRight90() {
  if (autoRunning) return;
  resetEncoders();
  runTurnStep(-turnPwmL, turnPwmR, "RE PHAI 90");
}

void cmdTurnLeft90() {
  if (autoRunning) return;
  resetEncoders();
  runTurnStep(turnPwmL, -turnPwmR, "RE TRAI 90");
}

void runBasicDriveTest() {
  if (autoRunning) return;
  autoRunning = true;
  flushSerial();
  Serial.println("\n>> TEST CO BAN: tien → lui → re phai 90 → re trai 90 → dung");

  if (!runStraightTest(1, "1/5 TIEN THANG 2s")) goto done;
  if (!runStepMs(0, 0, 600)) goto done;

  if (!runStraightTest(-1, "2/5 LUI THANG 2s")) goto done;
  if (!runStepMs(0, 0, 600)) goto done;

  resetEncoders();
  if (!runTurnStep(-turnPwmL, turnPwmR, "3/5 RE PHAI 90")) goto done;
  if (!runStepMs(0, 0, 600)) goto done;

  resetEncoders();
  if (!runTurnStep(turnPwmL, -turnPwmR, "4/5 RE TRAI 90")) goto done;
  if (!runStepMs(0, 0, 400)) goto done;

  cmdStop();
  beep(200);
  Serial.println(">> TEST CO BAN xong (5/5 DUNG)");
  printEncoders();

done:
  autoRunning = false;
}

void handleLineCommand(const String& raw) {
  String cmd = raw;
  cmd.trim();
  cmd.toLowerCase();
  if (cmd.length() == 0) return;

  if (cmd == "tien" || cmd == "fwd" || cmd == "forward" || cmd == "thang") {
    cmdForwardTest();
  } else if (cmd == "lui" || cmd == "rev" || cmd == "back") {
    cmdBackwardTest();
  } else if (cmd == "rephai" || cmd == "right" || cmd == "r90" || cmd == "phai90") {
    cmdTurnRight90();
  } else if (cmd == "retrai" || cmd == "left" || cmd == "l90" || cmd == "trai90") {
    cmdTurnLeft90();
  } else if (cmd == "dung" || cmd == "stop" || cmd == "halt") {
    cmdStop();
  } else if (cmd == "test" || cmd == "basic" || cmd == "coban") {
    runBasicDriveTest();
  } else if (cmd == "menu" || cmd == "help") {
    printMenu();
  } else if (cmd == "enc" || cmd == "encoder") {
    printEncoders();
  } else if (cmd == "balance" || cmd == "canbang") {
    runBalanceTest(false);
  } else {
    Serial.printf("Lenh '%s' khong hop le. Thu :tien :lui :rephai :retrai :dung :test\n",
                  cmd.c_str());
  }
}

void runDirectionTest() {
  if (autoRunning) return;
  autoRunning = true;
  flushSerial();

  Serial.println("\n=== TEST CHIEU BANH ===");
  Serial.println("Moi banh chay TIEN 2s — encoder phai TANG neu dung chieu.");
  Serial.printf("Sign hien tai: MOTOR_R=%d MOTOR_L=%d | ENC_R=%d ENC_L=%d\n",
                motorRSign, motorLSign, encRSign, encLSign);

  runWheelDirectionTest("PHAI TIEN (chi banh phai)", 0, testPwmR);
  delay(400);
  runWheelDirectionTest("TRAI TIEN (chi banh trai)", testPwmL, 0);
  delay(400);
  int left, right;
  straightDuties(1, left, right);
  runWheelDirectionTest("CA 2 TIEN", left, right);

  Serial.println("\nNeu banh chay lui nhung enc tang → doi MOTOR_x_SIGN (phim o/i).");
  Serial.println("Neu banh chay dung nhung enc giam → doi ENC_x_SIGN (phim u/j).");
  Serial.println("Xong → phim p de copy config.h");
  autoRunning = false;
}

void printMenu() {
  Serial.println();
  Serial.println("=== LENH DIEU KHIEN CO BAN ===");
  Serial.println("w / f     TIEN thang 2s (encoder)");
  Serial.println("k / b     LUI thang 2s (encoder)");
  Serial.println("n / t     RE PHAI 90 do (encoder)");
  Serial.println("l / y     RE TRAI 90 do (encoder)");
  Serial.println("0         DUNG");
  Serial.println("5 / 6     giu TIEN / LUI (nhan 0 de dung)");
  Serial.println("!         chay day du: tien→lui→re phai→re trai→dung");
  Serial.println(":tien :lui :rephai :retrai :dung :test  (go + Enter)");
  Serial.println();
  Serial.println("=== HIEN CHUAN / TUNE ===");
  Serial.println("1/2  PHAI  tien/lui");
  Serial.println("3/4  TRAI  tien/lui");
  Serial.println("5/6  CA 2  tien/lui (co trim)");
  Serial.println("7/8  xoay phai/trai (giu phim)");
  Serial.println("s/*  can bang PWM 2 banh (do encoder / * ap dung)");
  Serial.println("d    test chieu banh (tung banh + ca 2, 2s/buoc)");
  Serial.println("o/i  dao MOTOR_L_SIGN / MOTOR_R_SIGN");
  Serial.println("u/j  dao ENC_L_SIGN / ENC_R_SIGN");
  Serial.println("a    test day du tu dong (calib + tune)");
  Serial.println("m    (0,0)->(0,90) tien 90cm bang encoder + trim");
  Serial.println("c    calibrate: tien→lui→xoay→in config");
  Serial.println("p    in block config.h de copy");
  Serial.println("e    doc encoder  |  r reset encoder");
  Serial.println("?    xem lai menu");
  Serial.println("+/-  PWM ca 2 banh cung luc");
  Serial.println(";/'  PWM TRAI -/+5    ./  PWM PHAI -/+5");
  Serial.println("(/)  trimF tien -/+1   z/x  trimR lui -/+1");
  Serial.println("g/h  counts/cm -/+1  (dung thuoc met: counts/cm = progress / cm thuc te)");
  Serial.println("[/]  PWM xoay L   {/} PWM xoay R   9/, turn90");
  Serial.printf("PWM L/R=%d/%d trimF=%d trimR=%d turnL=%d turnR=%d turn90=%ld\n",
                testPwmL, testPwmR, straightTrimFwd, straightTrimRev,
                turnPwmL, turnPwmR, turn90Target);
  Serial.printf("Sign motor R/L=%d/%d  enc R/L=%d/%d  counts/cm=%.1f  path m=%dcm\n",
                motorRSign, motorLSign, encRSign, encLSign, countsPerCm, TEST_PATH_CM);
  Serial.println("=========================================");
}

void runCalibrationSuite() {
  flushSerial();
  autoRunning = true;
  Serial.println("\n>> CALIBRATE — tune (/) trimF, z/x trimR, roi chay lai c\n");

  if (!runStraightTest(1, "BUOC 1/4: CA 2 TIEN (co trim)")) goto done;
  if (!runStepMs(0, 0, 500)) goto done;

  if (!runStraightTest(-1, "BUOC 2/4: CA 2 LUI (co trim)")) goto done;
  if (!runStepMs(0, 0, 500)) goto done;

  resetEncoders();
  if (!runTurnStep(-turnPwmL, turnPwmR, "BUOC 3/4: XOAY PHAI 90")) goto done;
  if (!runStepMs(0, 0, 500)) goto done;

  resetEncoders();
  if (!runTurnStep(turnPwmL, -turnPwmR, "BUOC 4/4: XOAY TRAI 90")) goto done;

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

  if (!runStepMsBalanced(1, STRAIGHT_TEST_MS)) goto aborted;
  Serial.println(">> CA 2 TIEN"); printEncoders();

  if (!runStepMs(0, 0, 500)) goto aborted;

  if (!runStepMsBalanced(-1, STRAIGHT_TEST_MS)) goto aborted;
  Serial.println(">> CA 2 LUI"); printEncoders();

  if (!runStepMs(0, 0, 500)) goto aborted;

  if (!runStepMs(0, testPwmR, STRAIGHT_TEST_MS)) goto aborted;
  Serial.println(">> PHAI TIEN"); printEncoders();

  if (!runStepMs(0, 0, 500)) goto aborted;

  if (!runStepMs(testPwmL, 0, STRAIGHT_TEST_MS)) goto aborted;
  Serial.println(">> TRAI TIEN"); printEncoders();

  if (!runStepMs(0, 0, 500)) goto aborted;

  resetEncoders();
  if (!runTurnStep(-turnPwmL, turnPwmR, "XOAY PHAI 90")) goto aborted;
  if (!runStepMs(0, 0, 500)) goto aborted;

  resetEncoders();
  if (!runTurnStep(turnPwmL, -turnPwmR, "XOAY TRAI 90")) goto aborted;

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
    case '1':
      resetEncoders();
      setMotors(0, testPwmR);
      Serial.printf("PHAI tien PWM=%d\n", testPwmR);
      break;
    case '2':
      resetEncoders();
      setMotors(0, -testPwmR);
      Serial.printf("PHAI lui PWM=%d\n", testPwmR);
      break;
    case '3':
      resetEncoders();
      setMotors(testPwmL, 0);
      Serial.printf("TRAI tien PWM=%d\n", testPwmL);
      break;
    case '4':
      resetEncoders();
      setMotors(-testPwmL, 0);
      Serial.printf("TRAI lui PWM=%d\n", testPwmL);
      break;
    case '5': setMotorsBalanced(1);  Serial.println("CA 2 tien"); break;
    case '6': setMotorsBalanced(-1); Serial.println("CA 2 lui");  break;
    case '7': setMotors(-turnPwmL, turnPwmR); Serial.println("Xoay PHAI"); break;
    case '8': setMotors(turnPwmL, -turnPwmR); Serial.println("Xoay TRAI"); break;
    case 'w':
    case 'f': cmdForwardTest(); break;
    case 'k':
    case 'b': cmdBackwardTest(); break;
    case 'n':
    case 't': cmdTurnRight90(); break;
    case 'l':
    case 'y': cmdTurnLeft90(); break;
    case '!': if (!autoRunning) runBasicDriveTest(); break;
    case '0': cmdStop(); break;
    case 's': if (!autoRunning) runBalanceTest(false); break;
    case '*': if (!autoRunning) runBalanceTest(true); break;
    case 'd': if (!autoRunning) runDirectionTest(); break;
    case 'o': flipMotorSign(true); break;
    case 'i': flipMotorSign(false); break;
    case 'u': flipEncSign(true); break;
    case 'j': flipEncSign(false); break;
    case 'e': printEncoders(); break;
    case 'r': resetEncoders(); Serial.println("Reset encoder = 0"); break;
    case 'p': printConfigBlock(); break;
    case 'c': if (!autoRunning) runCalibrationSuite(); break;
    case '+':
    case '=': adjustBothPwm(10); break;
    case '-':
    case '_': adjustBothPwm(-10); break;
    case ';': testPwmL = constrain(testPwmL - 5, 30, MAX_PWM_DUTY);
              Serial.printf("PWM TRAI=%d\n", testPwmL); break;
    case '\'': testPwmL = constrain(testPwmL + 5, 30, MAX_PWM_DUTY);
               Serial.printf("PWM TRAI=%d\n", testPwmL); break;
    case '.': testPwmR = constrain(testPwmR - 5, 30, MAX_PWM_DUTY);
              Serial.printf("PWM PHAI=%d\n", testPwmR); break;
    case '/': testPwmR = constrain(testPwmR + 5, 30, MAX_PWM_DUTY);
              Serial.printf("PWM PHAI=%d\n", testPwmR); break;
    case '(': straightTrimFwd = constrain(straightTrimFwd - 1, 0, 40);
              Serial.printf("trimF=%d\n", straightTrimFwd); break;
    case ')': straightTrimFwd = constrain(straightTrimFwd + 1, 0, 40);
              Serial.printf("trimF=%d\n", straightTrimFwd); break;
    case '{': turnPwmR = constrain(turnPwmR - 5, 60, MAX_PWM_DUTY); Serial.printf("turnR=%d\n", turnPwmR); break;
    case '}': turnPwmR = constrain(turnPwmR + 5, 60, MAX_PWM_DUTY); Serial.printf("turnR=%d\n", turnPwmR); break;
    case 'z': straightTrimRev = constrain(straightTrimRev - 1, 0, 40);
              Serial.printf("trimR=%d\n", straightTrimRev); break;
    case 'x': straightTrimRev = constrain(straightTrimRev + 1, 0, 40);
              Serial.printf("trimR=%d\n", straightTrimRev); break;
    case '[': turnPwmL = constrain(turnPwmL - 5, 60, MAX_PWM_DUTY); Serial.printf("turnL=%d\n", turnPwmL); break;
    case ']': turnPwmL = constrain(turnPwmL + 5, 60, MAX_PWM_DUTY); Serial.printf("turnL=%d\n", turnPwmL); break;
    case '9': turn90Target = constrain(turn90Target + 50, 500, 4000);
              Serial.printf("turn90=%ld\n", turn90Target); break;
    case ',': turn90Target = constrain(turn90Target - 50, 500, 4000);
              Serial.printf("turn90=%ld\n", turn90Target); break;
    case 'g': countsPerCm = constrain(countsPerCm - 1.0f, 40.0f, 130.0f);
              Serial.printf("counts/cm=%.1f\n", countsPerCm); break;
    case 'h': countsPerCm = constrain(countsPerCm + 1.0f, 40.0f, 130.0f);
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
  pinMode(PIN_ENC_L_C1, INPUT_PULLUP);
  pinMode(PIN_ENC_L_C2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_R_C1), isrEncR, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_L_C1), isrEncL, RISING);
  resetEncoders();

  beep(100);
  Serial.println("\n=== ESP32 TANK — MOTOR TEST ===");
  Serial.println("Chay sketch nay truoc esp32-tank-jbg37.ino");
  Serial.println("Quy trinh: s/* can bang → w/k/n/l test co ban → p copy config.h");
  Serial.println("[v5] w/k/n/l/0 = tien/lui/re phai/re trai/dung | ! = test day du");
  Serial.printf("PWM can bang: TRAI=%d PHAI=%d (trimF=%d)\n",
                testPwmL, testPwmR, straightTrimFwd);
  printMenu();
}

void loop() {
  static String lineBuf;

  while (Serial.available()) {
    const char c = Serial.read();
    if (c == '\r') continue;

    if (c == '\n') {
      if (lineBuf.length() > 0) {
        if (lineBuf.charAt(0) == ':') {
          handleLineCommand(lineBuf.substring(1));
        } else {
          handleLineCommand(lineBuf);
        }
        lineBuf = "";
      }
      continue;
    }

    if (lineBuf.length() > 0 || c == ':') {
      lineBuf += c;
      continue;
    }

    handleCommand(c);
  }

  static unsigned long lastPrint = 0;
  if (autoRunning && millis() - lastPrint >= 500) {
    lastPrint = millis();
    printEncoders();
  }
}
