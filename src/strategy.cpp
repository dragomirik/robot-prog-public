#include "strategy.h"

FutureAction::FutureAction(
    Vector2 target,
    bool activeKicker) : _changeMove(true),
                         _target(target),
                         _activeKicker(activeKicker){};
FutureAction::FutureAction(
    bool activeKicker) : _changeMove(false),
                         _target(Vector2(0, 0)),
                         _activeKicker(activeKicker){};

////////
const int goalMinDistance = 75;
const FutureAction stopRobot = FutureAction(Vector2(0,0), false);
FutureAction chooseStrategy(FieldProperties fP, RobotState cS) {
  if (robotIsLost(fP, cS)) {

    if (!ballIsDetected(fP, cS)) {
      return stopRobot;

    } else if (ballIsCaught(fP, cS)) {
      if (!goalIsDetected(fP, cS)) {
        return stopRobot;
      } else if (targetJustInFrontOfRobot(fP, cS, cS.enemyGoalPos())) {
        return shootStrategy(fP, cS);
      } else {
        return accelerateToGoalStrategyWithCam(fP, cS);
      }

    } else {
      if (targetInFrontOfRobotFromFront(fP, cS, cS.ballPos())) {
        return goToBallStrategy(fP, cS);
      } else {
        return goToBallAvoidingBallStrategyWithCam(fP, cS);
      }
    }

  } else {

    if (leavingField(fP, cS)) {
      return refrainFromLeavingStrategy(fP, cS);

    } else if (!ballIsDetected(fP, cS)) {
      return slalowingBackwardsStrategy(fP, cS);

    } else if (ballIsCaught(fP, cS)) {
      if (targetJustInFrontOfRobot(fP, cS, cS.myPos().distanceRef(fP.enemyGoalPos()))) {
        return shootStrategy(fP, cS);
      } else {
        return accelerateToGoalStrategyWithLidar(fP, cS);
      }

    } else {
      if (targetInFrontOfRobotFromFront(fP, cS, cS.ballPos())) {
        return goToBallStrategy(fP, cS);
      } else {
        return goToBallAvoidingBallStrategyWithLidar(fP, cS);
      }
    }
  }
}

bool robotIsLost(FieldProperties fP, RobotState cS) {
  return cS.myPos() == Vector2(-9999,-9999);
}

bool leavingField(FieldProperties fP, RobotState cS) {
  SerialDebug.println("Left wall : " + String(cS.myPos().x() < -fP.fieldWidth() / 2 + fP.robotRadius())
  + " Right wall : " + String(fP.fieldWidth() / 2 - fP.robotRadius() < cS.myPos().x())
  + " Back wall : " + String(cS.myPos().y() < -fP.fieldLength() / 2 + fP.robotRadius())
  + " Front wall : " + String(fP.fieldLength() / 2 - fP.robotRadius() < cS.myPos().y())
  + " Enemy goal : " + String(cS.enemyGoalPos().norm() < goalMinDistance && cS.enemyGoalPos().norm() > 1)
  + " My goal : " + String(cS.myGoalPos().norm() < goalMinDistance && cS.myGoalPos().norm() > 1)  
  );

  SerialDebug.println(cS.myGoalPos().norm());

  // SerialDebug.println(cS.myPos().y());
  // SerialDebug.println(fP.fieldLength());
  
  return (cS.myPos().x() < -fP.fieldWidth() / 2 + fP.robotRadius()) ||
         (fP.fieldWidth() / 2 - fP.robotRadius() < cS.myPos().x()) ||
         (cS.myPos().y() < -fP.fieldLength() / 2 + fP.robotRadius()) ||
         (fP.fieldLength() / 2 - fP.robotRadius() < cS.myPos().y()) ||
         (cS.enemyGoalPos().norm() < goalMinDistance && cS.enemyGoalPos().norm() > 1) ||
         (cS.myGoalPos().norm() < goalMinDistance && cS.myGoalPos().norm() > 1);
}

bool ballIsDetected(FieldProperties fP, RobotState cS) {
  return cS.ballPos() != Vector2(0, 0);
}

bool goalIsDetected(FieldProperties fP, RobotState cS) {
  return cS.enemyGoalPos() != Vector2(0, 0);
}

bool targetInFrontOfRobotFromFront(FieldProperties fP, RobotState cS, Vector2 tL) {
  float longRobot = (fP.robotRadius() * 1.5);
  return tL.y() > longRobot;
}

bool targetInFrontOfRobotFromMiddle(FieldProperties fP, RobotState cS, Vector2 tL) {
  return tL.y() > 0;
}

bool targetCenterOfRobot(FieldProperties fP, RobotState cS, Vector2 tL) {
  return abs(tL.x()) <= 12;
}

bool targetJustInFrontOfRobot(FieldProperties fP, RobotState cS, Vector2 tL) {
  return targetInFrontOfRobotFromMiddle(fP, cS, tL) && targetCenterOfRobot(fP, cS, tL);
}

bool targetJustBehindOfRobot(FieldProperties fP, RobotState cS, Vector2 tL) {
  return (!targetInFrontOfRobotFromMiddle(fP, cS, tL)) && targetCenterOfRobot(fP, cS, tL);
}

bool ballIsCaught(FieldProperties fP, RobotState cS) {
  return targetJustInFrontOfRobot(fP, cS, cS.ballPos()) && cS.ballPos().y() <= 42;
}

FutureAction refrainFromLeavingStrategy(FieldProperties fP, RobotState cS) {
  SerialDebug.println("refrainFromLeavingStrategy");
  int xDirection = 0;
  int yDirection = 0;

  if (cS.myPos().x() < -fP.fieldWidth() / 2 + fP.robotRadius()) {
    xDirection = 10;
  } else if (fP.fieldWidth() / 2 - fP.robotRadius() < cS.myPos().x()) {
    xDirection = -10;
  }

  if (cS.myPos().y() < -fP.fieldLength() / 2 + fP.robotRadius()) {
    yDirection = 10;
  } else if (fP.fieldLength() / 2 - fP.robotRadius() < cS.myPos().y()) {
    yDirection = -10;
  }

  if (cS.enemyGoalPos().norm() < goalMinDistance) {
    yDirection = -10;
  } else if (cS.myGoalPos().norm() < goalMinDistance) {
    yDirection = 10;
  }

  return FutureAction(
      Vector2(
          xDirection,
          yDirection),
      false);
}

FutureAction goToBallStrategy(FieldProperties fP, RobotState cS) {
  SerialDebug.println("goToBallStrategy");
  return FutureAction(
      Vector2(
          cS.ballPos().x(),
          cS.ballPos().y() - fP.robotRadius() * 4),
      false);
}

FutureAction goToBallAvoidingBallStrategyWithCam(FieldProperties fP, RobotState cS) {
  SerialDebug.println("goToBallAvoidingBallStrategyWithCam");
  if (targetJustBehindOfRobot(fP, cS, cS.ballPos())) {
    return FutureAction(
        Vector2(10, -10),
        false);

  } else {
    return FutureAction(
          Vector2(0, -10),
          false);
  }
}

FutureAction goToBallAvoidingBallStrategyWithLidar(FieldProperties fP, RobotState cS) {
  SerialDebug.println("goToBallAvoidingBallStrategyWithLidar");
  if (targetJustBehindOfRobot(fP, cS, cS.ballPos())) {
    if (cS.myPos().x() < 0) {
      return FutureAction(
          Vector2(10, -10),
          false);

    } else {
      return FutureAction(
          Vector2(-10, -10),
          false);
    }

  } else {
    return FutureAction(
          Vector2(0, -10),
          false);
  }
}

FutureAction accelerateToGoalStrategyWithCam(FieldProperties fP, RobotState cS) {
  SerialDebug.println("accelerateToGoalStrategyWithCam");
  return FutureAction(
      Vector2(
          cS.enemyGoalPos().x(),
          cS.enemyGoalPos().y() - 15),
      false);
}

FutureAction accelerateToGoalStrategyWithLidar(FieldProperties fP, RobotState cS) {
  SerialDebug.println("accelerateToGoalStrategyWithLidar");
  return FutureAction(
      Vector2(
          fP.enemyGoalPos().x() - cS.myPos().x(),
          fP.enemyGoalPos().y() - cS.myPos().y() - 15),
      false);
}

FutureAction slalowingBackwardsStrategy(FieldProperties fP, RobotState cS) {
  SerialDebug.println("slalowingBackwardsStrategy");
  if (cS.myPos().y() < -70) {
    if (cS.myPos().x() < -5) {
      return FutureAction(
          Vector2(10, 0),
          false);
    } else if (5 < cS.myPos().x()) {
      return FutureAction(
          Vector2(-10, 0),
          false);
    } else {
      return FutureAction(
          Vector2(0, 10),
          false);
    }

  } else if (50 < cS.myPos().y()) {
    return FutureAction(
        Vector2(-20, -10),
        false);

  } else {
    if (cS.myPos().x() < -fP.fieldWidth() / 6) {
      return FutureAction(
          Vector2(20, -10),
          false);
    } else if (fP.fieldWidth() / 6 < cS.myPos().x()) {
      return FutureAction(
          Vector2(-20, -10),
          false);
    } else {
      return FutureAction(false);
    }
  }
}

FutureAction shootStrategy(FieldProperties fP, RobotState cS) {
  SerialDebug.println("shootStrategy");
  return FutureAction(
      Vector2(0, 20),
      true);
}
