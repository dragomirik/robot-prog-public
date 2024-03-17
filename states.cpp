#include "states.h"

FieldProperties::FieldProperties(
    float fieldLength,
    float fieldDepth,
    float spaceBeforeLineSide,
    float goalWidth)
    : _fieldLength(fieldLength),
      _fieldDepth(fieldDepth),
      _spaceBeforeLineSide(spaceBeforeLineSide),
      _goalWidth(goalWidth) {}

RobotState::RobotState(
    Vector2 ballPos,
    Vector2 myPos,
    Vector2 partnerPos)
    : _ballPos(ballPos),
      _myPos(myPos),
      _partnerPos(partnerPos) {}

RobotState RobotState::fromString(RobotState defaultValues, String values) {
  Vector2OrError b = RobotState::splitLastUpdate(values, 'b');
  Vector2OrError m = RobotState::splitLastUpdate(values, 'm');
  Vector2OrError p = RobotState::splitLastUpdate(values, 'p');
  return RobotState(
      b.defaultIfError(defaultValues.ballPos()),
      m.defaultIfError(defaultValues.myPos()),
      p.defaultIfError(defaultValues.partnerPos()));
}

Vector2OrError RobotState::splitLastUpdate(String values, char charId) {
  int pos = values.lastIndexOf(charId);
  if (pos == -1) {
    return Vector2OrError("error RobotState splitLastUpdate: no '" + String(charId) + "' found in '" + values + "'");
  } else {
    Vector2OrError val = RobotState::splitFirstVector(values.substring(pos + 1));
    if (val.isError()) {
      return Vector2OrError("error RobotState splitLastUpdate, error in splitFirstVector: " + val.errorMessage());
    } else {
      return val;
    }
  }
}

Vector2OrError RobotState::splitFirstVector(String part) {
  String x_num;
  String y_num;
  bool add_x = true;
  for (unsigned int i = 0; i < part.length(); i++) {
    char character = part.charAt(i);
    if (isDigit(character) || character == '.') {
      if (add_x) {
        x_num += character;
      } else {
        y_num += character;
      }
    } else if (character == ',') {
      if (add_x) {
        add_x = false;
      } else {
        return Vector2OrError("error RobotState splitFirstVector, several characters ','");
      }
    } else if (character == 'b' || character == 'm' || character == 'p') {
      break;
    } else {
      return Vector2OrError("error RobotState splitFirstVector, unknown character '" + String(character) + "'");
    }
  }
  return Vector2OrError(Vector2(
      x_num.toFloat(),
      y_num.toFloat()));
}

String RobotState::toString() const {
  return "RobotState (ballPos: " + _ballPos.toString() + " myPos: " + _myPos.toString() + " partnerPos " + _partnerPos.toString() + ")";
}
