#ifndef STRATEGY_H
#define STRATEGY_H

#include <Arduino.h>

#include "states.h"
#include "utilities.h"

class FutureAction {
 private:
  const Optional<MutableVector2> _target;
  const bool _activeKicker;
  const int _celerity;
  const Radians _orientation;

 public:
  FutureAction(
      Vector2 target,
      int celerity,
      Radians orientation,
      bool activeKicker);
  FutureAction(
      int celerity,
      Radians orientation,
      bool activeKicker);

  inline bool changeMove() const { return _target.hasValue(); }
  inline Vector2 target() const { return _target.value().toVector2(); }
  inline bool activeKicker() const { return _activeKicker; }
  inline int celerity() const { return _celerity; }
  inline Radians orientation() const { return _orientation; }
};

FutureAction chooseStrategy(FieldProperties fP, RobotState cS, double orientation, Vector2 nearestWall);

bool robotIsLost(FieldProperties fP, RobotState cS);
bool leavingField(FieldProperties fP, RobotState cS, Vector2 nearestWall);
bool targetInFrontOfRobotFromFront(FieldProperties fP, RobotState cS, Vector2 tL);
bool targetInFrontOfRobotFromMiddle(FieldProperties fP, RobotState cS, Vector2 tL);
bool targetCenterOfRobot(FieldProperties fP, RobotState cS, Vector2 tL);
bool targetJustInFrontOfRobot(FieldProperties fP, RobotState cS, Vector2 tL);
bool targetJustBehindOfRobot(FieldProperties fP, RobotState cS, Vector2 tL);
bool goalIsDetected(FieldProperties fP, RobotState cS);
bool ballIsDetected(FieldProperties fP, RobotState cS);
bool ballIsCaught(FieldProperties fP, RobotState cS);

FutureAction refrainFromLeavingStrategy(FieldProperties fP, RobotState cS, double orientation, Vector2 nearestWall);
FutureAction goToBallStrategy(FieldProperties fP, RobotState cS);
FutureAction goToBallAvoidingBallStrategyWithCam(FieldProperties fP, RobotState cS);
FutureAction goToBallAvoidingBallStrategyWithLidar(FieldProperties fP, RobotState cS);
FutureAction accelerateToGoalStrategyWithCam(FieldProperties fP, RobotState cS);
FutureAction accelerateToGoalStrategyWithLidar(FieldProperties fP, RobotState cS);
FutureAction slalowingBackwardsStrategy(FieldProperties fP, RobotState cS);
FutureAction shootStrategy(FieldProperties fP, RobotState cS);

#endif