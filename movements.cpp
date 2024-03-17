#include "movements.h"

///////MOTORMOV
MotorMov::MotorMov(
    uint8_t pinPWM,
    uint8_t pinCWCCW,
    uint8_t pinFG,
    float angleAxisKicker)
    : _pinPWM(pinPWM),
      _pinCWCCW(pinCWCCW),
      _pinFG(pinFG),
      _angleAxisKicker(angleAxisKicker * PI / 180) {
  pinMode(_pinPWM, OUTPUT);
  pinMode(_pinCWCCW, OUTPUT);
  pinMode(_pinFG, INPUT);
  _direction = Direction::stopped;
}

void MotorMov::stop() {
  _pwm(0);
  _direction = Direction::stopped;
}

void MotorMov::move(int value) {
  if (value == 0) {
    stop();
  } else if (value > 0) {
    // forward
    if (_direction == Direction::backward) {
      stop();
    }
    _cwccw(LOW);
    _pwm(value);
    _direction = Direction::forward;
  } else {
    // backward
    if (_direction == Direction::forward) {
      stop();
    }
    _cwccw(HIGH);
    _pwm(-value);
    _direction = Direction::backward;
  }
}

float MotorMov::anglePowerAxisKicker() const {
  return _angleAxisKicker - (PI / 2);
}

void MotorMov::_pwm(int value) const {
  analogWrite(_pinPWM, value);
}

void MotorMov::_cwccw(uint8_t value) const {
  digitalWrite(_pinCWCCW, value);
}

uint8_t MotorMov::_fg() const {
  return digitalRead(_pinFG);
}

///////MOTORS
Motors::Motors(
    MotorMov frontRight,
    MotorMov frontLeft,
    MotorMov backRight,
    MotorMov backLeft)
    : _frontRight(frontRight),
      _frontLeft(frontLeft),
      _backRight(backRight),
      _backLeft(backLeft) {}

void Motors::fullStop() {
  frontRight().stop();
  frontLeft().stop();
  backRight().stop();
  backLeft().stop();
}

void Motors::goTo(Vector2 distances, int celerity) {
  // If the distance to the destination is less than x, stop the motors
  if (sq(distances.x()) + sq(distances.y()) < sq(3)) {  // TODO faire de 3 un parametre global
    fullStop();
  } else {
    // If the ordinate is zero, the angle is 90°.
    float angle;
    if (distances.y() == 0) {
      angle = PI / 2;
    } else {
      angle = atan2(abs(distances.x()), abs(distances.y()));
    }

    // Change the angle according to the corner in which the destination point is located
    if (distances.x() <= 0 && distances.y() >= 0) {
      angle *= -1;
    } else if (distances.x() >= 0 && distances.y() <= 0) {
      angle = PI - angle;
    } else if (distances.x() <= 0 && distances.y() <= 0) {
      angle -= PI;
    }

    // The speed to be sent to the motors is calculated
    float MFRcelerity = cos(angle - frontRight().angleAxisKicker());
    float MFLcelerity = cos(angle - frontLeft().angleAxisKicker());
    float MBRcelerity = cos(angle - backRight().angleAxisKicker());
    float MBLcelerity = cos(angle - backLeft().angleAxisKicker());

    // The ratio to be used to calculate the speeds to be sent to the motors is calculated, taking into account the desired speed.
    float rapport = (celerity / 255) / (max(abs(MFRcelerity), max(abs(MFLcelerity), max(abs(MBRcelerity), abs(MBLcelerity)))));

    // Speeds are recalculated taking into account the desired speed and
    // Sends speeds to motors
    frontRight().move(MFRcelerity * rapport * 255);
    frontLeft().move(MFLcelerity * rapport * 255);
    backRight().move(MBRcelerity * rapport * 255);
    backLeft().move(MBLcelerity * rapport * 255);
  }
}