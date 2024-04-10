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
const int goalMinDistance = 100;
FutureAction chooseStrategy(FieldProperties fP, RobotState cS) {
  Vector2 bL = cS.ballPos();
  // Vector2 bL = cS.myPos().distanceRef(cS.ballPos());
  // Vector2 gL = cS.myPos().distanceRef(fP.enemyGoalPos());
  if (leavingField(fP, cS)) {
    return refrainFromLeavingStrategy(fP, cS);

  } else if (!ballIsDetected(fP, cS)) {
    return FutureAction(false);
    // return slalowingBackwardsStrategy(fP, cS);

  } else if (ballIsCaught(fP, cS, bL)) {
    return accelerateToGoalStrategy(fP, cS);
    // if (closeToShoot(fP, cS, gL)) {
    //   return shootStrategy(fP, cS);
    // } else {
    //   return accelerateToGoalStrategy(fP, cS);
    // }

  } else {
    if (targetInFrontOfRobot(fP, cS, bL)) {
      return goToBallStrategy(fP, cS);
    } else {
      return goToBallAvoidingBallStrategy(fP, cS, bL);
    }
  }
}

bool leavingField(FieldProperties fP, RobotState cS) {
  return (cS.myPos().x() < -fP.fieldWidth() / 2 + fP.robotRadius()) ||
         (fP.fieldWidth() / 2 - fP.robotRadius() < cS.myPos().x()) ||
         (cS.myPos().y() < -fP.fieldLength() / 2 + fP.robotRadius()) ||
         (fP.fieldLength() / 2 - fP.robotRadius() < cS.myPos().y()) ||
         (cS.enemyGoalPos().norm() < goalMinDistance && cS.enemyGoalPos().realNorm() > fP.robotRadius()) ||
         (cS.myGoalPos().norm() < goalMinDistance && cS.myGoalPos().realNorm() > fP.robotRadius());
  // (cS.nearestWallDistance > 0 && cS.nearestWallDistance < 300);
}

bool ballIsDetected(FieldProperties fP, RobotState cS) {
  return cS.ballPos() != fP.noneVect();
}

bool targetInFrontOfRobot(FieldProperties fP, RobotState cS, Vector2 tL) {
  float longRobot = (fP.robotRadius() + (fP.ballRadius() * 2)) * 1.5;
  return tL.y() > longRobot;
}

bool targetCenterOfRobot(FieldProperties fP, RobotState cS, Vector2 tL) {
  return abs(tL.x()) <= 10;
}

bool targetJustInFrontOfRobot(FieldProperties fP, RobotState cS, Vector2 tL) {
  return targetInFrontOfRobot(fP, cS, tL) && targetCenterOfRobot(fP, cS, tL);
}

bool targetJustBehindOfRobot(FieldProperties fP, RobotState cS, Vector2 tL) {
  return (!targetInFrontOfRobot(fP, cS, tL)) && targetCenterOfRobot(fP, cS, tL);
}

bool ballIsCaught(FieldProperties fP, RobotState cS, Vector2 bL) {
  return targetJustInFrontOfRobot(fP, cS, bL) && bL.realNorm() <= fP.robotRadius() + (2 * fP.ballRadius()) + 10;
}

bool closeToShoot(FieldProperties fP, RobotState cS, Vector2 gL) {
  return targetJustInFrontOfRobot(fP, cS, gL);
}

int getBallSidePositionFromRobot(FieldProperties fP, RobotState cS, Vector2 bL) {
  if (bL.x() < -fP.robotRadius() * 5) {
    return -1;
  } else if (fP.robotRadius() * 1.5 < bL.x()) {
    return 1;
  } else {
    return 0;
  }
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

  if ((cS.enemyGoalPos().norm() < goalMinDistance && cS.enemyGoalPos().realNorm() > fP.robotRadius()) ||
      (cS.myGoalPos().norm() < goalMinDistance && cS.myGoalPos().realNorm() > fP.robotRadius())) {
    yDirection = -10;
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
          cS.ballPos().y() - fP.robotRadius() * 5),
      false);
}

FutureAction goToBallAvoidingBallStrategy(FieldProperties fP, RobotState cS, Vector2 bL) {
  int distanceDevitement = fP.robotRadius() * 5;
  if (cS.ballPos().x() < 0) {
    if (getBallSidePositionFromRobot(fP, cS, bL) == -1) {
      SerialDebug.println("full gauche");
      return FutureAction(
          Vector2(
              cS.myPos().x(),
              cS.myPos().y() - distanceDevitement),
          false);

    } else if (bL.norm() < distanceDevitement) {
      SerialDebug.println("proche du robot");
      return FutureAction(
          Vector2(
              cS.myPos().x() + distanceDevitement,
              cS.myPos().y()),
          false);

    } else {
      SerialDebug.println("derriere le robot gauche");
      return FutureAction(
          Vector2(
              cS.myPos().x() + distanceDevitement,
              cS.myPos().y() - distanceDevitement),
          false);
    }

  } else {
    if (getBallSidePositionFromRobot(fP, cS, bL) == 1) {
      SerialDebug.println("full droite");
      return FutureAction(
          Vector2(
              cS.myPos().x(),
              cS.myPos().y() - distanceDevitement),
          false);

    } else if (bL.norm() < distanceDevitement) {
      SerialDebug.println("proche du robot");
      return FutureAction(
          Vector2(
              cS.myPos().x() - distanceDevitement,
              cS.myPos().y()),
          false);

    } else {
      SerialDebug.println("derriere le robot droite");
      return FutureAction(
          Vector2(
              cS.myPos().x() - distanceDevitement,
              cS.myPos().y() - distanceDevitement),
          false);
    }
  }
}

FutureAction accelerateToGoalStrategy(FieldProperties fP, RobotState cS) {
  SerialDebug.println("accelerateToGoalStrategy");
  return FutureAction(
      Vector2(
          fP.enemyGoalPos().x(),
          fP.enemyGoalPos().y() + 15),
      false);
  // return cS.enemyGoalPos();
}

FutureAction slalowingBackwardsStrategy(FieldProperties fP, RobotState cS) {
  SerialDebug.println("slalowingBackwardsStrategy");
  if (cS.myPos().y() < -70) {
    if (cS.myPos().x() < -5) {
      return FutureAction(
          Vector2(
              cS.myPos().x() + 10,
              cS.myPos().y()),
          false);
    } else if (5 < cS.myPos().x()) {
      return FutureAction(
          Vector2(
              cS.myPos().x() - 10,
              cS.myPos().y()),
          false);
    } else {
      return FutureAction(
          Vector2(
              cS.myPos().x(),
              cS.myPos().y() + 10),
          false);
    }

  } else if (50 < cS.myPos().y()) {
    return FutureAction(
        Vector2(
            cS.myPos().x() - 20,
            cS.myPos().y() - 10),
        false);

  } else {
    if (cS.myPos().x() < -fP.fieldWidth() / 6) {
      return FutureAction(
          Vector2(
              cS.myPos().x() + 20,
              cS.myPos().y() - 10),
          false);
    } else if (fP.fieldWidth() / 6 < cS.myPos().x()) {
      return FutureAction(
          Vector2(
              cS.myPos().x() - 20,
              cS.myPos().y() - 10),
          false);
    } else {
      // On ne change pas la direction du robot (ajouter une condition pour ne rien changer si noneVect est renvoyé)
      return FutureAction(false);
    }
  }
}

FutureAction shootStrategy(FieldProperties fP, RobotState cS) {
  SerialDebug.println("shootStrategy");
  // digitalWrite(KICKER, HIGH);
  // sleep(0.01);
  // digitalWrite(KICKER, LOW);
  return FutureAction(
      Vector2(0, 20),
      true);
}
