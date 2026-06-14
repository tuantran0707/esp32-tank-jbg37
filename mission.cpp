#include "mission.h"

MissionController::MissionController()
    : motors_(nullptr),
      odom_(nullptr),
      mode_(MODE_MANUAL),
      autoState_(AUTO_DISABLED),
      autoCfg_{false, 0, 0, 0},
      pose_{0, 0, HEAD_NORTH},
      countsPerCm_(DEFAULT_COUNTS_PER_CM),
      turn90Counts_(DEFAULT_TURN_90_COUNTS),
      scan360Ms_(DEFAULT_SCAN_360_MS),
      queueCount_(0),
      queueIndex_(0),
      stepRunning_(false),
      stepEndsAt_(0),
      stepStartEncR_(0),
      stepStartEncL_(0),
      missionActive_(false),
      scanRunning_(false),
      scanStartedAt_(0),
      lastGasSampleAt_(0),
      scanPeakGas_(-1),
      scanPeakAngle_(0),
      manualX_(0),
      manualY_(0),
      manualAutoStop_(false),
      manualStopAt_(0) {}

void MissionController::begin(TankMotors* motors, Odometry* odom) {
  motors_ = motors;
  odom_ = odom;
}

void MissionController::setCalibration(float countsPerCm, long turn90Counts, uint32_t scan360Ms) {
  countsPerCm_ = countsPerCm;
  turn90Counts_ = turn90Counts;
  scan360Ms_ = scan360Ms;
  if (odom_) {
    odom_->setCountsPerCm(countsPerCm_);
    odom_->setTurn90Counts(turn90Counts_);
  }
}

void MissionController::setMode(ControlMode mode) {
  mode_ = mode;
  if (mode_ == MODE_MANUAL) cancelAuto();
}

void MissionController::setAutoConfig(const AutoConfig& cfg) {
  autoCfg_ = cfg;
}

void MissionController::driveXY(int x, int y) {
  x = constrain(x, -100, 100);
  y = constrain(y, -100, 100);
  const int left  = (constrain(x + y, -100, 100) * MAX_PWM_DUTY) / 100;
  const int right = (constrain(x - y, -100, 100) * MAX_PWM_DUTY) / 100;
  motors_->setDuties(left, right);
  manualX_ = x;
  manualY_ = y;
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
  stepRunning_ = false;
}

void MissionController::cancelAuto() {
  missionActive_ = false;
  scanRunning_ = false;
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
  queuePush(TURN_PWM_LEFT, -TURN_PWM_RIGHT, turn90Counts_, true,
            (uint32_t)(turn90Counts_ * 4), 0, 0, +1);
}

void MissionController::queueAddTurnLeft90() {
  queuePush(-TURN_PWM_LEFT, TURN_PWM_RIGHT, turn90Counts_, true,
            (uint32_t)(turn90Counts_ * 4), 0, 0, -1);
}

void MissionController::queueTurnTo(Heading target) {
  const int diff = ((int)target - (int)pose_.heading + 4) % 4;
  if (diff == 0) return;
  if (diff == 1) queueAddTurnRight90();
  else if (diff == 3) queueAddTurnLeft90();
  else { queueAddTurnRight90(); queueAddTurnRight90(); }
}

void MissionController::queueAddForwardCm(int cm) {
  if (cm <= 0) return;
  const long encTarget = (long)(cm * countsPerCm_);
  const uint32_t timeout = (uint32_t)(cm * 80 + 500);

  int dx = 0, dy = 0;
  switch (pose_.heading) {
    case HEAD_NORTH: dy = cm; break;
    case HEAD_EAST:  dx = cm; break;
    case HEAD_SOUTH: dy = -cm; break;
    case HEAD_WEST:  dx = -cm; break;
  }
  queuePush(MAX_PWM_DUTY, MAX_PWM_DUTY, encTarget, false, timeout, dx, dy, 0);
}

void MissionController::queuePathTo(int dstX, int dstY) {
  queueClear();
  const Pose saved = pose_;

  const int dx = dstX - pose_.x;
  const int dy = dstY - pose_.y;

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

void MissionController::applyStepToPose(const MotionStep& step) {
  pose_.x = constrain(pose_.x + step.deltaX, 0, MAP_SIZE_CM);
  pose_.y = constrain(pose_.y + step.deltaY, 0, MAP_SIZE_CM);
  const int hd = ((int)pose_.heading + step.deltaHeading + 4) % 4;
  pose_.heading = (Heading)hd;
}

bool MissionController::encoderStepDone(const MotionStep& step) const {
  const long dR = odom_->getRightCount() - stepStartEncR_;
  const long dL = odom_->getLeftCount() - stepStartEncL_;

  if (step.isTurn) {
    const long progress = (dL - dR) / 2;
    if (dL <= 0 || dR >= 0) return false;
    return labs(progress) >= step.encTarget;
  }

  if (dR <= 0 || dL <= 0) return false;
  return ((dR + dL) / 2) >= step.encTarget;
}

bool MissionController::processQueue(unsigned long nowMs) {
  if (!stepRunning_ && queueIndex_ < queueCount_) {
    const MotionStep& st = queue_[queueIndex_];
    stepStartEncR_ = odom_->getRightCount();
    stepStartEncL_ = odom_->getLeftCount();
    motors_->setDuties(st.leftDuty, st.rightDuty);
    stepEndsAt_ = nowMs + st.timeoutMs;
    stepRunning_ = true;
  }

  if (stepRunning_) {
    const MotionStep& st = queue_[queueIndex_];
    if (encoderStepDone(st) || (long)(nowMs - stepEndsAt_) >= 0) {
      stopAll();
      applyStepToPose(st);
      queueIndex_++;
      stepRunning_ = false;
    }
  }

  return (!stepRunning_ && queueIndex_ >= queueCount_);
}

void MissionController::startAutoNow() {
  if (mode_ != MODE_AUTO) return;
  missionActive_ = true;
  autoState_ = AUTO_MOVING_TO_TARGET;
  pose_ = {0, 0, HEAD_NORTH};
  odom_->resetCounts();
  queuePathTo(constrain(autoCfg_.targetX, 0, MAP_SIZE_CM),
              constrain(autoCfg_.targetY, 0, MAP_SIZE_CM));
}

void MissionController::beginGasScan(unsigned long nowMs) {
  scanRunning_ = true;
  scanStartedAt_ = nowMs;
  lastGasSampleAt_ = 0;
  scanPeakGas_ = -1;
  scanPeakAngle_ = 0;
  motors_->setDuties(TURN_PWM_DUTY, -TURN_PWM_DUTY);
  autoState_ = AUTO_SCANNING_GAS;
}

void MissionController::processManualTimeout(unsigned long nowMs) {
  if (mode_ == MODE_MANUAL && manualAutoStop_ && nowMs >= manualStopAt_) {
    stopAll();
    manualAutoStop_ = false;
  }
}

void MissionController::processAuto(unsigned long nowMs, bool timeValid, uint32_t epoch) {
  if (mode_ != MODE_AUTO) return;

  if (!autoCfg_.enabled) {
    if (autoState_ != AUTO_DISABLED) cancelAuto();
    return;
  }

  if (!missionActive_) {
    if (autoCfg_.runAtEpoch == 0) {
      startAutoNow();
      return;
    }
    autoState_ = AUTO_WAITING_TIME;
    if (timeValid && epoch >= autoCfg_.runAtEpoch) startAutoNow();
    return;
  }

  if (autoState_ == AUTO_MOVING_TO_TARGET) {
    if (processQueue(nowMs)) beginGasScan(nowMs);
  } else if (autoState_ == AUTO_RETURNING_HOME) {
    if (processQueue(nowMs)) {
      stopAll();
      missionActive_ = false;
      autoState_ = AUTO_FINISHED;
      autoCfg_.enabled = false;
    }
  }
}

bool MissionController::processGasScan(unsigned long nowMs, int gasRaw,
                                       void (*onSample)(int, int),
                                       void (*onDone)(int, int)) {
  if (!scanRunning_) return false;

  const unsigned long elapsed = nowMs - scanStartedAt_;

  if (lastGasSampleAt_ == 0 || nowMs - lastGasSampleAt_ >= GAS_SCAN_SAMPLE_MS) {
    lastGasSampleAt_ = nowMs;
    const int angle = (int)((elapsed * 360UL) / scan360Ms_) % 360;
    if (gasRaw > scanPeakGas_) {
      scanPeakGas_ = gasRaw;
      scanPeakAngle_ = angle;
    }
    if (onSample) onSample(gasRaw, angle);
  }

  if (elapsed >= scan360Ms_) {
    stopAll();
    scanRunning_ = false;
    if (onDone) onDone(scanPeakGas_, scanPeakAngle_);
    queuePathTo(0, 0);
    autoState_ = AUTO_RETURNING_HOME;
    return true;
  }
  return false;
}
