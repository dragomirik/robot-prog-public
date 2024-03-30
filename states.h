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
      float goalWidth,
      Vector2 myGoalPos,
      Vector2 enemyGoalPos,
      float robotRadius,
      float ballRadius);

  inline float fieldLength() const { return _fieldLength; }
  inline float fieldDepth() const { return _fieldDepth; }
  inline float spaceBeforeLineSide() const { return _spaceBeforeLineSide; }
  inline float goalWidth() const { return _goalWidth; }
  inline Vector2 myGoalPos() const { return _myGoalPos; }
  inline Vector2 enemyGoalPos() const { return _enemyGoalPos; }
  inline float robotRadius() const { return _robotRadius; }
  inline float ballRadius() const { return _ballRadius; }

 private:
  const float _fieldLength;
  const float _fieldDepth;
  const float _spaceBeforeLineSide;
  const float _goalWidth;
  const Vector2 _myGoalPos;
  const Vector2 _enemyGoalPos;
  const float _robotRadius;
  const float _ballRadius;
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

  bool updateFromString(char &typeState, String &xReadingState, String &yReadingState, bool &writingInXState, char newChar);

  inline Vector2 ballPos() const { return _ballPos.toVector2(); }
  inline Vector2 myPos() const { return _myPos.toVector2(); }
  inline Vector2 partnerPos() const { return _partnerPos.toVector2(); }

  String toString() const;

 private:
  MutableVector2 _ballPos, _myPos, _partnerPos;
};

#endif