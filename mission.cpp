#include "mission.h"

// Chu ky vong kin = 50Hz, giong delay(20) trong motor_test (chong dao dong)
#define CONTROL_PERIOD_MS 20UL

#ifndef STRAIGHT_STEP_TIMEOUT_MS_PER_CM
#define STRAIGHT_STEP_TIMEOUT_MS_PER_CM 95
#define STRAIGHT_STEP_TIMEOUT_EXTRA_MS  2000
#endif

#ifndef PATH_COMP_Y_CM
#define PATH_COMP_Y_CM  15
#endif
#ifndef PATH_COMP_X_CM
#define PATH_COMP_X_CM -15
#endif
#ifndef PWM_DUTY_LEFT
#define PWM_DUTY_LEFT MAX_PWM_DUTY
#endif
#ifndef PWM_DUTY_RIGHT
#define PWM_DUTY_RIGHT MAX_PWM_DUTY
#endif

#ifndef TURN_90_LEFT_COUNTS
#define TURN_90_LEFT_COUNTS  DEFAULT_TURN_90_COUNTS
#endif
#ifndef TURN_90_RIGHT_COUNTS
#define TURN_90_RIGHT_COUNTS DEFAULT_TURN_90_COUNTS
#endif
#ifndef TURN_180_COUNTS
#define TURN_180_COUNTS      (TURN_90_LEFT_COUNTS + TURN_90_RIGHT_COUNTS)
#endif
#ifndef STRAIGHT_ENC_KP
#define STRAIGHT_ENC_KP        2
#endif
#ifndef STRAIGHT_ENC_MAX_TRIM
#define STRAIGHT_ENC_MAX_TRIM  45
#endif
#ifndef TURN_PWM_MIN
#define TURN_PWM_MIN           85
#endif
#ifndef TURN_DECEL_ZONE_PCT
#define TURN_DECEL_ZONE_PCT    25
#endif
#ifndef PWM_MIN_DUTY
#define PWM_MIN_DUTY           70
#endif

#ifndef STRAIGHT_ENC_KI
#define STRAIGHT_ENC_KI 1
#endif
#ifndef STRAIGHT_DRIFT_TRIM
#define STRAIGHT_DRIFT_TRIM 0
#endif
#ifndef STRAIGHT_ENC_I_MAX
#define STRAIGHT_ENC_I_MAX 600L
#endif
#ifndef STRAIGHT_DECEL_ZONE_PCT
#define STRAIGHT_DECEL_ZONE_PCT 12
#endif
#ifndef STRAIGHT_PWM_FLOOR
#define STRAIGHT_PWM_FLOOR 110
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

MissionController::MissionController()
    : motors_(nullptr),
      odom_(nullptr),
      mode_(MODE_MANUAL),
      autoState_(AUTO_DISABLED),
      autoCfg_{0, 0, 0},
      pose_{0, 0, HEAD_NORTH},
      countsPerCm_(DEFAULT_COUNTS_PER_CM),
      turn90Counts_(DEFAULT_TURN_90_COUNTS),
      turn90LeftCounts_(TURN_90_LEFT_COUNTS),
      turn90RightCounts_(TURN_90_RIGHT_COUNTS),
      turn180Counts_(TURN_180_COUNTS),
      scan360Ms_(DEFAULT_SCAN_360_MS),
      mapSizeX_(MAP_SIZE_CM),
      mapSizeY_(MAP_SIZE_CM),
      queueCount_(0),
      queueIndex_(0),
      stepRunning_(false),
      stepEndsAt_(0),
      stepStartEncR_(0),
      stepStartEncL_(0),
      straightEncIntegral_(0),
      stepStartMs_(0),
      lastEncLoopMs_(0),
      missionActive_(false),
      manualX_(0),
      manualY_(0),
      manualAutoStop_(false),
      manualStopAt_(0),
      manualEncMoveActive_(false),
      dwellEndsAtMs_(0),
      lastTriggeredEpoch_(0) {}

void MissionController::begin(TankMotors* motors, Odometry* odom) {
  motors_ = motors;
  odom_ = odom;
}

void MissionController::setCalibration(float countsPerCm, long turn90Counts, uint32_t scan360Ms) {
  countsPerCm_ = countsPerCm;
  turn90Counts_ = turn90Counts;
  turn90LeftCounts_ = TURN_90_LEFT_COUNTS;
  turn90RightCounts_ = TURN_90_RIGHT_COUNTS;
  turn180Counts_ = TURN_180_COUNTS;
  scan360Ms_ = scan360Ms;
  if (odom_) {
    odom_->setCountsPerCm(countsPerCm_);
    odom_->setTurn90Counts(turn90Counts_);
  }
}

int MissionController::clampMotorDuty(int duty) {
  if (duty == 0) return 0;
  const int s = duty > 0 ? 1 : -1;
  return s * constrain(abs(duty), PWM_MIN_DUTY, MAX_PWM_DUTY);
}

void MissionController::applyStraightTrim(int& left, int& right) const {
  if (left > 0 && right > 0 && left == right) {
    const int scale = left;
    left = (PWM_DUTY_LEFT * scale) / MAX_PWM_DUTY;
    right = (PWM_DUTY_RIGHT * scale) / MAX_PWM_DUTY;
  } else if (left < 0 && right < 0 && left == right) {
    const int scale = -left;
    left = -((PWM_DUTY_LEFT * scale) / MAX_PWM_DUTY);
    right = -((PWM_DUTY_RIGHT * scale) / MAX_PWM_DUTY);
  }
}

void MissionController::setMapSize(int sizeX, int sizeY) {
  mapSizeX_ = constrain(sizeX, 10, 500);
  mapSizeY_ = constrain(sizeY, 10, 500);
}

void MissionController::setMode(ControlMode mode) {
  if (mode == MODE_MANUAL && mode_ == MODE_AUTO) cancelAuto();
  mode_ = mode;
  if (mode_ == MODE_AUTO && !missionActive_ && autoState_ == AUTO_DISABLED) {
    autoState_ = AUTO_WAITING_TIME;
  }
}

void MissionController::setAutoConfig(const AutoConfig& cfg) {
  if (cfg.runAtEpoch != autoCfg_.runAtEpoch) lastTriggeredEpoch_ = 0;
  autoCfg_ = cfg;
}

void MissionController::driveXY(int x, int y) {
  x = constrain(x, -100, 100);
  y = constrain(y, -100, 100);
  // y = tien/lui, x = xoay: trai = y+x, phai = y-x
  int left  = (constrain(y + x, -100, 100) * MAX_PWM_DUTY) / 100;
  int right = (constrain(y - x, -100, 100) * MAX_PWM_DUTY) / 100;
  applyStraightTrim(left, right);
  motors_->setDuties(left, right);
  manualX_ = x;
  manualY_ = y;
  manualEncMoveActive_ = false;
}

void MissionController::startManualMoveTo(int targetX, int targetY) {
  targetX = constrain(targetX, 0, mapSizeX_);
  targetY = constrain(targetY, 0, mapSizeY_);

  if (targetX == pose_.x && targetY == pose_.y) {
    return;
  }

  motors_->stop();
  manualX_ = 0;
  manualY_ = 0;
  manualAutoStop_ = false;
  queueClear();
  stepRunning_ = false;
  manualEncMoveActive_ = true;
  queuePathTo(targetX, targetY);
}

void MissionController::processManualEncMove(unsigned long nowMs) {
  if (!manualEncMoveActive_) return;
  if (processQueue(nowMs)) {
    manualEncMoveActive_ = false;
    stopAll();
  }
}

void MissionController::setManualDurationMs(int durationMs, unsigned long nowMs) {
  if (durationMs > 0) {
    manualAutoStop_ = true;
    manualStopAt_ = nowMs + (unsigned long)durationMs;
  } else {
    manualAutoStop_ = false;
  }
}

void MissionController::stopAll() {
  motors_->stop();
  manualX_ = 0;
  manualY_ = 0;
  manualAutoStop_ = false;
  manualEncMoveActive_ = false;
  stepRunning_ = false;
  queueClear();
}

void MissionController::cancelAuto() {
  missionActive_ = false;
  queueClear();
  autoState_ = AUTO_DISABLED;
  stopAll();
}

bool MissionController::queuePush(int l, int r, long encTarget, bool isTurn,
                                  uint32_t timeoutMs, int dx, int dy, int dh) {
  if (queueCount_ >= 32) return false;
  queue_[queueCount_] = {l, r, encTarget, isTurn, timeoutMs, dx, dy, dh};
  queueCount_++;
  return true;
}

void MissionController::queueClear() {
  queueCount_ = 0;
  queueIndex_ = 0;
  stepRunning_ = false;
}

void MissionController::queueAddTurnRight90() {
  // Re PHAI that = banh trai LUI, banh phai TIEN (da xac minh bang thuc nghiem)
  queuePush(-TURN_PWM_LEFT, TURN_PWM_RIGHT, turn90RightCounts_, true,
            (uint32_t)(turn90RightCounts_ * 5), 0, 0, +1);
}

void MissionController::queueAddTurnLeft90() {
  // Re TRAI that = banh trai TIEN, banh phai LUI
  queuePush(TURN_PWM_LEFT, -TURN_PWM_RIGHT, turn90LeftCounts_, true,
            (uint32_t)(turn90LeftCounts_ * 5), 0, 0, -1);
}

void MissionController::queueAddTurn180() {
  queuePush(-TURN_PWM_LEFT, TURN_PWM_RIGHT, turn180Counts_, true,
            (uint32_t)(turn180Counts_ * 5), 0, 0, -2);
}

void MissionController::queueTurnTo(Heading target) {
  const int diff = ((int)target - (int)pose_.heading + 4) % 4;
  if (diff == 0) return;
  // diff==1: heading +1 (vd N->E) = quay PHAI | diff==3: heading -1 = quay TRAI
  if (diff == 1) queueAddTurnRight90();
  else if (diff == 3) queueAddTurnLeft90();
  else queueAddTurn180();
}

void MissionController::queueAddForwardCm(int cm) {
  if (cm <= 0) return;

  int driveCm = cm;
  switch (pose_.heading) {
    case HEAD_NORTH:
    case HEAD_SOUTH:
      driveCm = cm + PATH_COMP_Y_CM;
      break;
    case HEAD_EAST:
    case HEAD_WEST:
      driveCm = cm + PATH_COMP_X_CM;
      break;
  }
  if (driveCm < 1) driveCm = 1;

  const long encTarget = (long)(driveCm * countsPerCm_);
  const uint32_t timeout = (uint32_t)(driveCm * STRAIGHT_STEP_TIMEOUT_MS_PER_CM
                                      + STRAIGHT_STEP_TIMEOUT_EXTRA_MS);

  int dx = 0, dy = 0;
  switch (pose_.heading) {
    case HEAD_NORTH: dy = cm; break;
    case HEAD_EAST:  dx = cm; break;
    case HEAD_SOUTH: dy = -cm; break;
    case HEAD_WEST:  dx = -cm; break;
  }
  int leftDuty = PWM_DUTY_LEFT;
  int rightDuty = PWM_DUTY_RIGHT;
  applyStraightTrim(leftDuty, rightDuty);
  queuePush(leftDuty, rightDuty, encTarget, false, timeout, dx, dy, 0);
}

void MissionController::queuePathTo(int dstX, int dstY) {
  queueClear();
  const Pose saved = pose_;

  dstX = constrain(dstX, 0, mapSizeX_);
  dstY = constrain(dstY, 0, mapSizeY_);

  const int dx = dstX - pose_.x;
  const int dy = dstY - pose_.y;

  // Manhattan: Y trước (0,0→0,100), rồi X (0,100→100,100)
  if (dy > 0) {
    queueTurnTo(HEAD_NORTH);
    pose_.heading = HEAD_NORTH;
    queueAddForwardCm(dy);
    pose_.y += dy;
  } else if (dy < 0) {
    queueTurnTo(HEAD_SOUTH);
    pose_.heading = HEAD_SOUTH;
    queueAddForwardCm(-dy);
    pose_.y += dy;
  }

  if (dx > 0) {
    queueTurnTo(HEAD_EAST);
    pose_.heading = HEAD_EAST;
    queueAddForwardCm(dx);
    pose_.x += dx;
  } else if (dx < 0) {
    queueTurnTo(HEAD_WEST);
    pose_.heading = HEAD_WEST;
    queueAddForwardCm(-dx);
    pose_.x += dx;
  }

  pose_ = saved;
}

void MissionController::queueReturnHome() {
  queueClear();
  const Pose saved = pose_;
  const int px = pose_.x;
  const int py = pose_.y;

  queueAddTurn180();
  pose_.heading = (Heading)(((int)pose_.heading + 2) % 4);

  if (px > 0) {
    queueTurnTo(HEAD_WEST);
    pose_.heading = HEAD_WEST;
    queueAddForwardCm(px);
    pose_.x = 0;
  }

  if (py > 0) {
    queueTurnTo(HEAD_SOUTH);
    pose_.heading = HEAD_SOUTH;
    queueAddForwardCm(py);
    pose_.y = 0;
  }

  pose_ = saved;
}

void MissionController::applyStepToPose(const MotionStep& step) {
  pose_.x = constrain(pose_.x + step.deltaX, 0, mapSizeX_);
  pose_.y = constrain(pose_.y + step.deltaY, 0, mapSizeY_);
  const int hd = ((int)pose_.heading + step.deltaHeading + 4) % 4;
  pose_.heading = (Heading)hd;
}

bool MissionController::encoderStepDone(const MotionStep& step, unsigned long nowMs) const {
  const long dR = odom_->getRightCount() - stepStartEncR_;
  const long dL = odom_->getLeftCount() - stepStartEncL_;

  if (step.isTurn) {
    if (nowMs - stepStartMs_ < TURN_MIN_RUN_MS) return false;
    const long progress = labs((dL - dR) / 2);
    return progress >= step.encTarget;
  }

  const long fwdR = (step.rightDuty > 0) ? max(0L, dR) : max(0L, -dR);
  const long fwdL = (step.leftDuty > 0) ? max(0L, dL) : max(0L, -dL);
  return ((fwdR + fwdL) / 2) >= step.encTarget;
}

static int turnPwmScale(long progress, long encTarget, int basePwm) {
  const long cruiseEnd = (encTarget * TURN_CRUISE_PCT) / 100;
  const long decelZone = max(1L, (encTarget * TURN_DECEL_ZONE_PCT) / 100);
  const long remaining = encTarget - progress;

  if (progress < cruiseEnd) return basePwm;

  if (remaining <= 0 || remaining >= decelZone) return basePwm;

  const int sc = (int)max((long)TURN_PWM_MIN, remaining * 100 / decelZone);
  return max((int)TURN_PWM_MIN, basePwm * sc / 100);
}

void MissionController::updateEncClosedLoop(const MotionStep& step) {
  const long dR = odom_->getRightCount() - stepStartEncR_;
  const long dL = odom_->getLeftCount() - stepStartEncL_;

  if (!step.isTurn) {
    const long fwdR = (step.rightDuty > 0) ? max(0L, dR) : max(0L, -dR);
    const long fwdL = (step.leftDuty > 0) ? max(0L, dL) : max(0L, -dL);
    const long progress = (fwdR + fwdL) / 2;
    const long err = (dL - dR) + (dR * STRAIGHT_DRIFT_TRIM) / 1000;

    straightEncIntegral_ += err;
    straightEncIntegral_ =
        constrain(straightEncIntegral_, -STRAIGHT_ENC_I_MAX, STRAIGHT_ENC_I_MAX);

    int corr = (int)(err * STRAIGHT_ENC_KP)
             + (int)(straightEncIntegral_ * STRAIGHT_ENC_KI / 1000);
    corr = constrain(corr, -STRAIGHT_ENC_MAX_TRIM, STRAIGHT_ENC_MAX_TRIM);

    static unsigned long lastStraightDbg = 0;
    const unsigned long nowDbg = millis();
    if (nowDbg - lastStraightDbg >= 300) {
      lastStraightDbg = nowDbg;
      Serial.printf("[STR] dL=%ld dR=%ld err=%ld corr=%d prog=%ld/%ld\n",
                    dL, dR, err, corr, progress, step.encTarget);
    }

    int pwmL = abs(step.leftDuty);
    int pwmR = abs(step.rightDuty);
    const int sL = step.leftDuty >= 0 ? 1 : -1;
    const int sR = step.rightDuty >= 0 ? 1 : -1;

    const long decelZone =
        max(1L, (step.encTarget * STRAIGHT_DECEL_ZONE_PCT) / 100);
    const long remaining = step.encTarget - progress;
    if (remaining > 0 && remaining < decelZone) {
      const int sc = (int)max(55L, remaining * 100 / decelZone);
      pwmL = max((int)STRAIGHT_PWM_FLOOR, pwmL * sc / 100);
      pwmR = max((int)STRAIGHT_PWM_FLOOR, pwmR * sc / 100);
    }

    const int newL = clampMotorDuty(sL * (pwmL - corr));
    const int newR = clampMotorDuty(sR * (pwmR + corr));
    motors_->setDuties(newL, newR);
    return;
  }

  const long progress = labs((dL - dR) / 2);
  const long magL = labs(dL);
  const long magR = labs(dR);
  int corr = (int)((magL - magR) * TURN_ENC_KP);
  corr = constrain(corr, -TURN_ENC_MAX_TRIM, TURN_ENC_MAX_TRIM);

  int pwmL = turnPwmScale(progress, step.encTarget, abs(step.leftDuty));
  int pwmR = turnPwmScale(progress, step.encTarget, abs(step.rightDuty));

  if (magL < magR) pwmL += corr;
  else if (magR < magL) pwmR += corr;

  const int sL = step.leftDuty >= 0 ? 1 : -1;
  const int sR = step.rightDuty >= 0 ? 1 : -1;
  motors_->setDuties(clampMotorDuty(sL * pwmL), clampMotorDuty(sR * pwmR));
}

bool MissionController::processQueue(unsigned long nowMs) {
  if (!stepRunning_ && queueIndex_ < queueCount_) {
    const MotionStep& st = queue_[queueIndex_];
    stepStartEncR_ = odom_->getRightCount();
    stepStartEncL_ = odom_->getLeftCount();
    straightEncIntegral_ = 0;
    stepStartMs_ = nowMs;
    lastEncLoopMs_ = nowMs - CONTROL_PERIOD_MS;  // chay vong kin ngay lan dau
    motors_->setDuties(st.leftDuty, st.rightDuty);
    stepEndsAt_ = nowMs + st.timeoutMs;
    stepRunning_ = true;
  }

  if (stepRunning_) {
    const MotionStep& st = queue_[queueIndex_];
    // Khoa vong kin ve 50Hz (20ms) giong motor_test. Cap nhat qua nhanh
    // (>1kHz trong loop chinh) gay dao dong corr ±60 -> xe lac, chay cham.
    if ((long)(nowMs - lastEncLoopMs_) >= (long)CONTROL_PERIOD_MS) {
      lastEncLoopMs_ = nowMs;
      updateEncClosedLoop(st);
    }
    if (encoderStepDone(st, nowMs) || (long)(nowMs - stepEndsAt_) >= 0) {
      motors_->stop();
      applyStepToPose(st);
      queueIndex_++;
      stepRunning_ = false;
    }
  }

  return (!stepRunning_ && queueIndex_ >= queueCount_);
}

void MissionController::startAutoNow() {
  if (mode_ != MODE_AUTO) return;
  if (missionActive_) return;
  missionActive_ = true;
  autoState_ = AUTO_MOVING_TO_TARGET;
  pose_ = {0, 0, HEAD_NORTH};
  odom_->resetCounts();
  queuePathTo(constrain(autoCfg_.xAuto, 0, mapSizeX_),
              constrain(autoCfg_.yAuto, 0, mapSizeY_));
}

void MissionController::startReturnHome() {
  if (mode_ != MODE_AUTO || missionActive_) return;
  missionActive_ = true;
  autoState_ = AUTO_RETURNING;
  queueReturnHome();
  queueIndex_ = 0;
  stepRunning_ = false;
}

void MissionController::finishReachTarget(unsigned long nowMs) {
  motors_->stop();
  missionActive_ = false;
  autoState_ = AUTO_AT_TARGET;
  dwellEndsAtMs_ = nowMs + AUTO_DWELL_MS;
}

void MissionController::finishReturnHome() {
  stopAll();
  missionActive_ = false;
  pose_ = {0, 0, HEAD_NORTH};
  odom_->resetCounts();
  autoState_ = (mode_ == MODE_AUTO) ? AUTO_WAITING_TIME : AUTO_FINISHED;
}

void MissionController::processManualTimeout(unsigned long nowMs) {
  if (mode_ == MODE_MANUAL && manualAutoStop_ && nowMs >= manualStopAt_) {
    stopAll();
    manualAutoStop_ = false;
  }
}

void MissionController::processAuto(unsigned long nowMs, bool timeValid, uint32_t epoch) {
  if (mode_ != MODE_AUTO) return;

  if (missionActive_) {
    if (autoState_ == AUTO_MOVING_TO_TARGET || autoState_ == AUTO_RETURNING) {
      if (processQueue(nowMs)) {
        if (autoState_ == AUTO_MOVING_TO_TARGET) {
          finishReachTarget(nowMs);
        } else {
          finishReturnHome();
        }
      }
    }
    return;
  }

  if (autoState_ == AUTO_AT_TARGET) {
    if ((long)(nowMs - dwellEndsAtMs_) >= 0) startReturnHome();
    return;
  }

  if (autoState_ == AUTO_FINISHED) return;

  autoState_ = AUTO_WAITING_TIME;
  if (autoCfg_.runAtEpoch > 0 && timeValid && epoch >= autoCfg_.runAtEpoch
      && autoCfg_.runAtEpoch != lastTriggeredEpoch_) {
    lastTriggeredEpoch_ = autoCfg_.runAtEpoch;
    startAutoNow();
  }
}
