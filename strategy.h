#ifndef STRATEGY_H
#define STRATEGY_H

#include <Arduino.h>

#include "states.h"
#include "utilities.h"

class FutureAction {
 private:
  const bool _changeMove;
  const Vector2 _goToPos;
  const bool _activeKicker;

 public:
  FutureAction(
      Vector2 goToPos,
      bool activeKicker);
  FutureAction(
      bool activeKicker);

  inline bool changeMove() const { return _changeMove; }
  inline Vector2 goToPos() const { return _goToPos; }
  inline bool activeKicker() const { return _activeKicker; }
};

FutureAction chooseStrategy(FieldProperties fP, RobotState cS);

bool leavingField(FieldProperties fP, RobotState cS);
bool targetInFrontOfRobot(FieldProperties fP, RobotState cS, Vector2 tL);
bool targetJustInFrontOfRobot(FieldProperties fP, RobotState cS, Vector2 tL);
bool targetJustBehindOfRobot(FieldProperties fP, RobotState cS, Vector2 tL);
bool ballIsDetected(FieldProperties fP, RobotState cS);
bool ballIsCaught(FieldProperties fP, RobotState cS, Vector2 bL);
bool closeToShoot(FieldProperties fP, RobotState cS, Vector2 gL);
int getSidePosition(FieldProperties fP, RobotState cS);
int getBallSidePositionFromRobot(FieldProperties fP, RobotState cS, Vector2 bL);

FutureAction refrainFromLeavingStrategy(FieldProperties fP, RobotState cS);
FutureAction goToBallStrategy(FieldProperties fP, RobotState cS);
FutureAction goToBallAvoidingBallStrategy(FieldProperties fP, RobotState cS, Vector2 bL);
FutureAction accelerateToGoalStrategy(FieldProperties fP, RobotState cS);
FutureAction slalowingBackwardsStrategy(FieldProperties fP, RobotState cS);
FutureAction shootStrategy(FieldProperties fP, RobotState cS);

#endif