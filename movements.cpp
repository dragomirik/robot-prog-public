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
  //pinMode(_pinFG, INPUT);
  stop();
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
    if (isLeft()) {
      _cwccw(LOW);
    } else {
      _cwccw(HIGH);
    }
    _pwm(value);
    _direction = Direction::forward;
  } else {
    // backward
    if (_direction == Direction::forward) {
      stop();
    }
    if (isLeft()) {
      _cwccw(HIGH);
    } else {
      _cwccw(LOW);
    }
    _pwm(-value);
    _direction = Direction::backward;
  }
}

float MotorMov::anglePowerAxisKicker() const {
  return _angleAxisKicker - (PI / 2);
}

void MotorMov::_pwm(int value) const {
  analogWrite(_pinPWM, 255 - value);
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

void Motors::fullStop() const {
  frontRight().stop();
  frontLeft().stop();
  backRight().stop();
  backLeft().stop();
}

void Motors::goTo(Vector2 vector, int celerity) const {
  // If the distance to the destination is less than x, stop the motors
  if (sq(vector.x()) + sq(vector.y()) < sq(3)) {  // TODO faire de 3 un parametre global
    fullStop();
  } else {
    // If the y value is zero, the angle is 90°.
    float angle;
    if (vector.y() == 0) {
      angle = PI / 2;
    } else {
      angle = atan2(abs(vector.x()), abs(vector.y()));
    }

    // Change the angle according to the corner in which the destination point is located
    if (vector.x() <= 0 && vector.y() >= 0) {
      angle *= -1;
    } else if (vector.x() >= 0 && vector.y() <= 0) {
      angle = PI - angle;
    } else if (vector.x() <= 0 && vector.y() <= 0) {
      angle -= PI;
    }

    // The speed to be sent to the motors is calculated
    float MFRcelerity = cos(angle - frontRight().angleAxisKicker());
    float MFLcelerity = cos(angle - frontLeft().angleAxisKicker());
    float MBRcelerity = -cos(angle - backRight().angleAxisKicker());
    float MBLcelerity = -cos(angle - backLeft().angleAxisKicker());
    
    // The ratio to be used to calculate the speeds to be sent to the motors is calculated, taking into account the desired speed.
    float max = (max(abs(MFRcelerity), max(abs(MFLcelerity), max(abs(MBRcelerity), abs(MBLcelerity)))));
    // SerialDebug.println("max : " + String(max));
    // SerialDebug.println("angle : " + String(angle));
    // SerialDebug.println("frontRight().angleAxisKicker() : " + String(frontRight().angleAxisKicker()));
    // SerialDebug.println("frontLeft().angleAxisKicker() : " + String(frontLeft().angleAxisKicker()));
    // SerialDebug.println("backRight().angleAxisKicker() : " + String(backRight().angleAxisKicker()));
    // SerialDebug.println("backLeft().angleAxisKicker() : " + String(backLeft().angleAxisKicker()));

    float rapport = (celerity / 255.0) / max;

    // SerialDebug.println("rapport : " + String(rapport));

    // Speeds are recalculated taking into account the desired speed and
    // Sends speeds to motors
    float speedFR = MFRcelerity * rapport * 255;
    float speedFL = MFLcelerity * rapport * 255;
    float speedBR = MBRcelerity * rapport * 255;
    float speedBL = MBLcelerity * rapport * 255;

    // SerialDebug.println(String(MFRcelerity) + ", speed=" + String(speedFR));
    // SerialDebug.println(String(MFLcelerity) + ", speed=" + String(speedFL));
    // SerialDebug.println(String(MBRcelerity) + ", speed=" + String(speedBR));
    // SerialDebug.println(String(MBLcelerity) + ", speed=" + String(speedBL));
    // SerialDebug.println("************");

// frontLeft().move(50);
// delay(2000);
// frontLeft().move(-50);
// delay(2000);
// frontLeft().stop();

// frontRight().move(50);
// delay(2000);
// frontRight().move(-50);
// delay(2000);
// frontRight().stop();

// backRight().move(50);
// delay(2000);
// backRight().move(-50);
// delay(2000);
// backRight().stop();

// backLeft().move(50);
// delay(2000);
// backLeft().move(-50);
// delay(2000);
// backLeft().stop();

    frontRight().move(speedFR);
    frontLeft().move(speedFL);
    backRight().move(speedBR);
    backLeft().move(speedBL);
  }
}