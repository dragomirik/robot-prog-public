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

      // SerialDebug.println("change to " + newMutableVector2.toString());
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