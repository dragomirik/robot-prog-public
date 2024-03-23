#ifndef STATES_H
#define STATES_H

#include <Arduino.h>
#include "utilities.h"


class FieldProperties {
public:
  FieldProperties(
    float fieldLength,
    float fieldDepth,
    float spaceBeforeLineSide,
    float goalWidth);

  inline float fieldLength() const { return _fieldLength; }
  inline float fieldDepth() const { return _fieldDepth; }
  inline float spaceBeforeLineSide() const { return _spaceBeforeLineSide; }
  inline float goalWidth() const { return _goalWidth; }

private:
  const float _fieldLength;
  const float _fieldDepth;
  const float _spaceBeforeLineSide;
  const float _goalWidth;
};


class RobotState {
public:
  RobotState(
    Vector2 ballPos,
    Vector2 myPos,
    Vector2 partnerPos);

  static RobotState fromString(RobotState defaultValues, String values);

  static Vector2OrError splitLastUpdate(String values, char charId);
  static Vector2OrError splitFirstVector(String part);

  inline Vector2 ballPos() const { return _ballPos; }
  inline Vector2 myPos() const { return _myPos; }
  inline Vector2 partnerPos() const { return _partnerPos; }

  String toString() const;

private:
  const Vector2 _ballPos, _myPos, _partnerPos;
};

#endif