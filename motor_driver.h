#pragma once

#include <Arduino.h>
#include "config.h"

class MotorDriver {
public:
  MotorDriver(int pwmPin, int in1Pin, int in2Pin, int sign, int pwmChannel);

  void begin();
  void setDuty(int duty);

private:
  int pwmPin_;
  int in1Pin_;
  int in2Pin_;
  int sign_;
  int pwmChannel_;

  void writeRaw(int pwm, bool forward);
};

class TankMotors {
public:
  void begin();
  void setDuties(int left, int right);
  void stop();

private:
  MotorDriver right_;
  MotorDriver left_;

public:
  TankMotors();
};
