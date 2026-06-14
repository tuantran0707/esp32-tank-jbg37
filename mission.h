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
  AUTO_SCANNING_GAS,
  AUTO_RETURNING_HOME,
  AUTO_FINISHED
};

struct AutoConfig {
  bool enabled;
  int targetX;
  int targetY;
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
  void setMode(ControlMode mode);
  void setAutoConfig(const AutoConfig& cfg);
  AutoConfig autoConfig() const { return autoCfg_; }

  ControlMode mode() const { return mode_; }
  AutoState autoState() const { return autoState_; }
  Pose pose() const { return pose_; }

  void driveXY(int x, int y);
  void setManualDurationMs(int durationMs, unsigned long nowMs);
  void stopAll();
  void cancelAuto();
  void startAutoNow();

  void processManualTimeout(unsigned long nowMs);
  void processAuto(unsigned long nowMs, bool timeValid, uint32_t epoch);
  bool processGasScan(unsigned long nowMs, int gasRaw,
                      void (*onSample)(int gas, int angle),
                      void (*onDone)(int peakGas, int peakAngle));
  void beginGasScan(unsigned long nowMs);

  bool manualTimeoutActive() const { return manualAutoStop_; }
  bool missionActive() const { return missionActive_; }
  bool scanRunning() const { return scanRunning_; }

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
  uint32_t scan360Ms_;

  MotionStep queue_[32];
  uint8_t queueCount_;
  uint8_t queueIndex_;
  bool stepRunning_;
  unsigned long stepEndsAt_;
  long stepStartEncR_;
  long stepStartEncL_;

  bool missionActive_;
  bool scanRunning_;
  unsigned long scanStartedAt_;
  unsigned long lastGasSampleAt_;
  int scanPeakGas_;
  int scanPeakAngle_;

  int manualX_;
  int manualY_;
  bool manualAutoStop_;
  unsigned long manualStopAt_;

  bool queuePush(int l, int r, long encTarget, bool isTurn,
                 uint32_t timeoutMs, int dx, int dy, int dh);
  void queueClear();
  void queueAddTurnRight90();
  void queueAddTurnLeft90();
  void queueTurnTo(Heading target);
  void queueAddForwardCm(int cm);
  bool processQueue(unsigned long nowMs);
  bool encoderStepDone(const MotionStep& step) const;
  void applyStepToPose(const MotionStep& step);
};
