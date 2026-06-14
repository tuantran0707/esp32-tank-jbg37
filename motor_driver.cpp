#include "motor_driver.h"

#define PWM_CH_RIGHT 0
#define PWM_CH_LEFT  1

TankMotors::TankMotors()
    : right_(PIN_MOTOR_R_PWM, PIN_MOTOR_R_IN1, PIN_MOTOR_R_IN2, MOTOR_R_SIGN, PWM_CH_RIGHT),
      left_(PIN_MOTOR_L_PWM, PIN_MOTOR_L_IN1, PIN_MOTOR_L_IN2, MOTOR_L_SIGN, PWM_CH_LEFT) {}

MotorDriver::MotorDriver(int pwmPin, int in1Pin, int in2Pin, int sign, int pwmChannel)
    : pwmPin_(pwmPin), in1Pin_(in1Pin), in2Pin_(in2Pin), sign_(sign), pwmChannel_(pwmChannel) {}

void MotorDriver::begin() {
  pinMode(in1Pin_, OUTPUT);
  pinMode(in2Pin_, OUTPUT);
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  ledcAttach(pwmPin_, PWM_FREQ_HZ, PWM_RES_BITS);
#else
  ledcSetup(pwmChannel_, PWM_FREQ_HZ, PWM_RES_BITS);
  ledcAttachPin(pwmPin_, pwmChannel_);
#endif
  writeRaw(0, true);
}

void MotorDriver::setDuty(int duty) {
  duty = constrain(duty * sign_, -MAX_PWM_DUTY, MAX_PWM_DUTY);
  const bool forward = duty >= 0;
  writeRaw(abs(duty), forward);
}

void MotorDriver::writeRaw(int pwm, bool forward) {
  digitalWrite(in1Pin_, forward ? HIGH : LOW);
  digitalWrite(in2Pin_, forward ? LOW : HIGH);
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  ledcWrite(pwmPin_, constrain(pwm, 0, PWM_MAX));
#else
  ledcWrite(pwmChannel_, constrain(pwm, 0, PWM_MAX));
#endif
}

void TankMotors::begin() {
  right_.begin();
  left_.begin();
}

void TankMotors::setDuties(int left, int right) {
  left_.setDuty(left);
  right_.setDuty(right);
}

void TankMotors::stop() {
  setDuties(0, 0);
}
