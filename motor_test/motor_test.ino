/*
 * TEST MOTOR + ENCODER — ESP32 Tank
 *
 * Dùng TRƯỚC khi nạp esp32-tank-jbg37.ino
 * Encoder: ngắt RISING (giống code xe cân bằng) — KHÔNG cần ESP32Encoder
 *
 * Board: ESP32 Dev Module | Serial 115200
 */

// --- Chân giống config.h ---
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

// Dấu đã tune trên xe cân bằng
#define MOTOR_R_SIGN  1
#define MOTOR_L_SIGN -1
#define ENC_R_SIGN    1
#define ENC_L_SIGN   -1

#define PWM_FREQ_HZ  20000
#define PWM_RES_BITS 8
#define PWM_CH_RIGHT 0
#define PWM_CH_LEFT  1

volatile long encCountR = 0;
volatile long encCountL = 0;

int testPwm = 120;
int turnPwmL = 165;   // trai: encoder tang cham hon -> PWM cao hon
int turnPwmR = 120;   // phai: encoder tang nhanh -> PWM thap hon
long turn90Target = 1650L;
bool autoRunning = false;

long turnProgress(long startL, long startR) {
  const long dL = encCountL - startL;
  const long dR = encCountR - startR;
  return (dL - dR) / 2;
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
  return true;
}

bool runTurnStep(int left, int right, const char* label) {
  const long startL = encCountL;
  const long startR = encCountR;
  Serial.printf(">> XOAY: %s  (L=%d R=%d, target=%ld counts)\n",
                label, left, right, turn90Target);
  setMotors(left, right);
  beep(50);

  unsigned long t0 = millis();
  while (millis() - t0 < 8000UL) {  // timeout 8s
    delay(20);
    const long prog = turnProgress(startL, startR);
    if (labs(prog) >= turn90Target) {
      Serial.printf("   Dat target sau %lu ms, progress=%ld\n",
                    millis() - t0, prog);
      break;
    }
    if (Serial.available()) {
      flushSerial();
      stopMotors();
      return false;
    }
  }
  stopMotors();
  printEncoders();
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

// -------------------------------------------------------------------
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
  Serial.printf("Enc PHAI: %ld  |  Enc TRAI: %ld\n", encCountR, encCountL);
}

void resetEncoders() {
  noInterrupts();
  encCountR = 0;
  encCountL = 0;
  interrupts();
}

void printMenu() {
  Serial.println();
  Serial.println("=== MENU TEST MOTOR ===");
  Serial.println("1/2  PHAI  tien/lui");
  Serial.println("3/4  TRAI  tien/lui");
  Serial.println("5/6  CA 2  tien/lui");
  Serial.println("7/8  xoay phai/trai");
  Serial.println("0    DUNG");
  Serial.println("e    in encoder");
  Serial.println("r    reset encoder ve 0");
  Serial.println("a    tu dong test day du");
  Serial.println("t    xoay phai 90 do (encoder)");
  Serial.println("y    xoay trai 90 do (encoder)");
  Serial.println("+/-  PWM tien/lui");
  Serial.println("[/]  PWM xoay trai/phai");
  Serial.printf("PWM=%d  turnL=%d turnR=%d  turn90=%ld\n",
                testPwm, turnPwmL, turnPwmR, turn90Target);
  Serial.println("=======================");
}

void runAutoTest() {
  flushSerial();
  autoRunning = true;
  Serial.println("\n>> AUTO TEST bat dau (nhan phim bat ky de dung)");

  if (!runStepMs(testPwm, testPwm, 2000)) goto aborted;
  Serial.println(">> CA 2 TIEN"); printEncoders();

  if (!runStepMs(0, 0, 500)) goto aborted;

  if (!runStepMs(-testPwm, -testPwm, 2000)) goto aborted;
  Serial.println(">> CA 2 LUI"); printEncoders();

  if (!runStepMs(0, 0, 500)) goto aborted;

  if (!runStepMs(0, testPwm, 2000)) goto aborted;
  Serial.println(">> PHAI TIEN"); printEncoders();

  if (!runStepMs(0, 0, 500)) goto aborted;

  if (!runStepMs(testPwm, 0, 2000)) goto aborted;
  Serial.println(">> TRAI TIEN"); printEncoders();

  if (!runStepMs(0, 0, 500)) goto aborted;

  if (!runTurnStep(turnPwmL, -turnPwmR, "XOAY PHAI 90")) goto aborted;
  if (!runStepMs(0, 0, 500)) goto aborted;
  if (!runTurnStep(-turnPwmL, turnPwmR, "XOAY TRAI 90")) goto aborted;

  stopMotors();
  autoRunning = false;
  beep(200);
  Serial.println(">> AUTO TEST xong.");
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
    case '5': setMotors(testPwm, testPwm);   Serial.println("CA 2 tien"); break;
    case '6': setMotors(-testPwm, -testPwm); Serial.println("CA 2 lui");  break;
    case '7': setMotors(turnPwmL, -turnPwmR); Serial.println("Xoay PHAI"); break;
    case '8': setMotors(-turnPwmL, turnPwmR); Serial.println("Xoay TRAI"); break;
    case '0': stopMotors(); Serial.println("DUNG"); break;
    case 'e': printEncoders(); break;
    case 'r': resetEncoders(); Serial.println("Reset encoder = 0"); break;
    case 't': runTurnStep(turnPwmL, -turnPwmR, "PHAI 90"); break;
    case 'y': runTurnStep(-turnPwmL, turnPwmR, "TRAI 90"); break;
    case '+':
    case '=': testPwm = constrain(testPwm + 10, 30, 255); Serial.printf("PWM=%d\n", testPwm); break;
    case '-':
    case '_': testPwm = constrain(testPwm - 10, 30, 255); Serial.printf("PWM=%d\n", testPwm); break;
    case '[': turnPwmL = constrain(turnPwmL - 5, 80, 255); Serial.printf("turnL=%d\n", turnPwmL); break;
    case ']': turnPwmL = constrain(turnPwmL + 5, 80, 255); Serial.printf("turnL=%d\n", turnPwmL); break;
    case '{': turnPwmR = constrain(turnPwmR - 5, 80, 255); Serial.printf("turnR=%d\n", turnPwmR); break;
    case '}': turnPwmR = constrain(turnPwmR + 5, 80, 255); Serial.printf("turnR=%d\n", turnPwmR); break;
    case '9': turn90Target = constrain(turn90Target + 50, 500, 4000);
              Serial.printf("turn90=%ld\n", turn90Target); break;
    case ',': turn90Target = constrain(turn90Target - 50, 500, 4000);
              Serial.printf("turn90=%ld\n", turn90Target); break;
    case 'a': if (!autoRunning) runAutoTest(); break;
    case '?': printMenu(); break;
    default: break;
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
  Serial.println("\n=== ESP32 TANK — TEST MOTOR ===");
  Serial.println("Kiem tra: TB6612 STBY noi 3.3V, nguon VM 6-12V, GND chung");
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
