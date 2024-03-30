#include "states.h"

FieldProperties::FieldProperties(
    float fieldLength,
    float fieldDepth,
    float spaceBeforeLineSide,
    float goalWidth,
    Vector2 myGoalPos,
    Vector2 enemyGoalPos,
    float robotRadius,
    float ballRadius)
    : _fieldLength(fieldLength),
      _fieldDepth(fieldDepth),
      _spaceBeforeLineSide(spaceBeforeLineSide),
      _goalWidth(goalWidth),
      _myGoalPos(myGoalPos),
      _enemyGoalPos(enemyGoalPos),
      _robotRadius(robotRadius),
      _ballRadius(ballRadius) {}

RobotState::RobotState(
    Vector2 ballPos,
    Vector2 myPos,
    Vector2 partnerPos)
    : _ballPos(ballPos),
      _myPos(myPos),
      _partnerPos(partnerPos) {}

bool RobotState::updateFromString(char &typeState, String &xReadingState, String &yReadingState, bool &writingInXState, char newChar) {
  if (newChar == 'b' || newChar == 'm' || newChar == 'p') {
    if (xReadingState != "" && yReadingState != "") {
      MutableVector2 newMutableVector2 = MutableVector2(Vector2(
          xReadingState.toFloat(),
          yReadingState.toFloat()));

      //SerialDebug.println("change to " + newMutableVector2.toString());
      if (typeState == 'b') {
        _ballPos = newMutableVector2;
      } else if (typeState == 'm') {
        _myPos = newMutableVector2;
      } else if (typeState == 'p') {
        _partnerPos = newMutableVector2;
      }
    } else {
      SerialDebug.println("ERROR CATCHED RobotState: unfinished data : '" + xReadingState + " , " + yReadingState + "'");
    }
    typeState = newChar;
    xReadingState = "";
    yReadingState = "";
    writingInXState = true;
    return true;
  } else if (!(typeState == 'b' || typeState == 'm' || typeState == 'p')) {
    SerialDebug.println("ERROR CATCHED RobotState: no typeState tracked");
  } else if (isDigit(newChar) || newChar == '.' || newChar == '-') {
    if (writingInXState) {
      xReadingState += newChar;
    } else {
      yReadingState += newChar;
    }
  } else if (newChar == ',') {
    if (writingInXState) {
      writingInXState = false;
    } else {
      SerialDebug.println("ERROR CATCHED RobotState : several characters ','");
      xReadingState = "";
      yReadingState = "";
      writingInXState = true;
      typeState = 'x';
    }
  } else {
    SerialDebug.println("ERROR CATCHED RobotState : unknown char (skipped)");
  }
  return false;
}

// TODO change actual detection system to a char by char one
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
    if (isDigit(character) || character == '.' || character == '-') {
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
/*
String RobotState::toString() const {
  return "RobotState (ballPos: " + _ballPos.toString() + " myPos: " + _myPos.toString() + " partnerPos " + _partnerPos.toString() + ")";
}*/

String RobotState::toString() const {
  String result = "RobotState (ballPos: ";
  result += _ballPos.toString();
  result += " myPos: ";
  result += _myPos.toString();
  result += " partnerPos: ";
  result += _partnerPos.toString();
  result += ")";
  return result;
}