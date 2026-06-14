#include "odometry.h"

static volatile long gEncCountR = 0;
static volatile long gEncCountL = 0;

void IRAM_ATTR isrEncRight() {
  if (digitalRead(PIN_ENC_R_C2)) gEncCountR += ENC_R_SIGN;
  else                           gEncCountR -= ENC_R_SIGN;
}

void IRAM_ATTR isrEncLeft() {
  if (digitalRead(PIN_ENC_L_C2)) gEncCountL += ENC_L_SIGN;
  else                           gEncCountL -= ENC_L_SIGN;
}

Odometry::Odometry()
    : countsPerCm_(DEFAULT_COUNTS_PER_CM),
      turn90Counts_(DEFAULT_TURN_90_COUNTS) {}

void Odometry::begin() {
  pinMode(PIN_ENC_R_C1, INPUT_PULLUP);
  pinMode(PIN_ENC_R_C2, INPUT_PULLUP);
  pinMode(PIN_ENC_L_C1, INPUT);  // GPIO34/35: trở kéo 10k ngoài
  pinMode(PIN_ENC_L_C2, INPUT);

  attachInterrupt(digitalPinToInterrupt(PIN_ENC_R_C1), isrEncRight, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_L_C1), isrEncLeft, RISING);
  resetCounts();
}

void Odometry::resetCounts() {
  noInterrupts();
  gEncCountR = 0;
  gEncCountL = 0;
  interrupts();
}

long Odometry::getRightCount() const { return gEncCountR; }
long Odometry::getLeftCount() const { return gEncCountL; }
long Odometry::getAvgCount() const {
  return (getRightCount() + getLeftCount()) / 2;
}

long Odometry::countsForCm(int cm) const {
  return (long)(fabsf((float)cm) * countsPerCm_);
}
