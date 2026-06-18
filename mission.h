#pragma once

#include <Arduino.h>
#include "config.h"
#include "odometry.h"
#include "motor_driver.h"

enum ControlMode { MODE_MANUAL = 0, MODE_AUTO = 1 };

enum AutoState {
  AUTO_DISABLED = 0,
  AUTO_WAITING_TIME,
  AUTO_MOVING_TO_TARGET,
  AUTO_AT_TARGET,
  AUTO_RETURNING,
  AUTO_FINISHED
};

struct AutoConfig {
  int xAuto;
  int yAuto;
  uint32_t runAtEpoch;
};

struct MotionStep {
  int leftDuty;
  int rightDuty;
  long encTarget;
  bool isTurn;
  uint32_t timeoutMs;
  int deltaX;
  int deltaY;
  int deltaHeading;
};

class MissionController {
public:
  void begin(TankMotors* motors, Odometry* odom);

  void setCalibration(float countsPerCm, long turn90Counts, uint32_t scan360Ms);
  void setMapSize(int sizeX, int sizeY);
  int mapSizeX() const { return mapSizeX_; }
  int mapSizeY() const { return mapSizeY_; }
  void setMode(ControlMode mode);
  void setAutoConfig(const AutoConfig& cfg);
  AutoConfig autoConfig() const { return autoCfg_; }

  ControlMode mode() const { return mode_; }
  AutoState autoState() const { return autoState_; }
  Pose pose() const { return pose_; }

  void driveXY(int x, int y);
  void startManualMoveTo(int targetX, int targetY);
  void setManualDurationMs(int durationMs, unsigned long nowMs);
  void stopAll();
  void cancelAuto();
  void startAutoNow();
  void startReturnHome();

  void processManualTimeout(unsigned long nowMs);
  void processManualEncMove(unsigned long nowMs);
  void processAuto(unsigned long nowMs, bool timeValid, uint32_t epoch);

  bool manualTimeoutActive() const { return manualAutoStop_; }
  bool manualEncMoveActive() const { return manualEncMoveActive_; }
  bool missionActive() const { return missionActive_; }

  void queuePathTo(int dstX, int dstY);

  MissionController();

private:
  TankMotors* motors_;
  Odometry* odom_;

  ControlMode mode_;
  AutoState autoState_;
  AutoConfig autoCfg_;
  Pose pose_;

  float countsPerCm_;
  long turn90Counts_;
  long turn90LeftCounts_;
  long turn90RightCounts_;
  long turn180Counts_;
  uint32_t scan360Ms_;
  int mapSizeX_;
  int mapSizeY_;

  void applyStraightTrim(int& left, int& right) const;
  void finishReachTarget(unsigned long nowMs);
  void finishReturnHome();
  void queueReturnHome();

  MotionStep queue_[32];
  uint8_t queueCount_;
  uint8_t queueIndex_;
  bool stepRunning_;
  unsigned long stepEndsAt_;
  long stepStartEncR_;
  long stepStartEncL_;
  long straightEncIntegral_;
  unsigned long stepStartMs_;
  unsigned long lastEncLoopMs_;   // khoa vong kin ve 50Hz (giong motor_test)

  bool missionActive_;
  int manualX_;
  int manualY_;
  bool manualAutoStop_;
  unsigned long manualStopAt_;
  bool manualEncMoveActive_;
  unsigned long dwellEndsAtMs_;
  uint32_t lastTriggeredEpoch_;

  bool queuePush(int l, int r, long encTarget, bool isTurn,
                 uint32_t timeoutMs, int dx, int dy, int dh);
  void queueClear();
  void queueAddTurnRight90();
  void queueAddTurnLeft90();
  void queueAddTurn180();
  void queueTurnTo(Heading target);
  void queueAddForwardCm(int cm);
  bool processQueue(unsigned long nowMs);
  bool encoderStepDone(const MotionStep& step, unsigned long nowMs) const;
  void updateEncClosedLoop(const MotionStep& step);
  static int clampMotorDuty(int duty);
  void applyStepToPose(const MotionStep& step);
};
