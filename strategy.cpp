#include "strategy.h"

Vector2 chooseStrategy(FieldProperties fP, RobotState cS) {
  Vector2 bL = cS.myPos().distanceRef(cS.ballPos());
  Vector2 gL = cS.myPos().distanceRef(fP.enemyGoalPos());
  if (!ballIsDetected(fP, cS, bL)) {
    return slalowingBackwardsStrategy(fP, cS, bL);
  } else if (ballIsCaught(fP, cS, bL)) {
    if (closeToShoot(fP, cS, gL)) {
      return shootStrategy(fP, cS, bL);
    } else {
      return accelerateToGoalStrategy(fP, cS, gL);
    }
  } else {
    if (targetInFrontOfRobot(fP, cS, bL)) {
      return goToBallStrategy(fP, cS, bL);
    } else {
      return goToBallAvoidingBallStrategy(fP, cS, bL);
    }
  }
}

bool targetInFrontOfRobot(FieldProperties fP, RobotState cS, Vector2 tL) {
  // We consider that this margin percentage is required
  float longRobot = (fP.robotRadius() + (fP.ballRadius() * 2)) * 1.5;
  return tL.y() > longRobot;
}

bool targetCenterOfRobot(FieldProperties fP, RobotState cS, Vector2 tL) {
  return abs(tL.x()) <= 5;
}

bool targetJustInFrontOfRobot(FieldProperties fP, RobotState cS, Vector2 tL) {
  return targetInFrontOfRobot(fP, cS, tL) && targetCenterOfRobot(fP, cS, tL);
}

bool targetJustBehindOfRobot(FieldProperties fP, RobotState cS, Vector2 tL) {
  return (!targetInFrontOfRobot(fP, cS, tL)) && targetCenterOfRobot(fP, cS, tL);
}

bool ballIsCaught(FieldProperties fP, RobotState cS, Vector2 bL) {
  return targetInFrontOfRobot(fP, cS, bL) && bL.norm() <= fP.robotRadius() + fP.ballRadius() + 6;
}

bool closeToShoot(FieldProperties fP, RobotState cS, Vector2 gL) {
  return targetInFrontOfRobot(fP, cS, gL) && gL.norm() <= 30;
}

int getSidePosition(FieldProperties fP, RobotState cS) {
  if (cS.myPos().x() < 0) {
    return -1;
  } else {
    return 1;
  }
}

Vector2 goToBallStrategy(FieldProperties fP, RobotState cS) {
  return Vector2(
      cS.ballPos().x(),
      cS.ballPos().y() - fP.robotRadius() * 1.5);
}

Vector2 goToBallAvoidingBallStrategy(FieldProperties fP, RobotState cS) {
  if (getSidePosition(fP, cS) == -1) {
    return Vector2(
        cS.ballPos().x() + fP.robotRadius() + 6,
        cS.ballPos().y() - fP.robotRadius() * 1.5);
  } else {
    return Vector2(
        cS.ballPos().x() - fP.robotRadius() + 6,
        cS.ballPos().y() - fP.robotRadius() * 1.5);
  }
}

Vector2 accelerateToGoalStrategy(FieldProperties fP, RobotState cS) {
  return Vector2(
      fP.enemyGoalPos().x(),
      fP.enemyGoalPos().y() - 15);
}