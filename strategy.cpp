#include "strategy.h"

Vector2 chooseStrategy(FieldProperties fP, RobotState cS) {
  Vector2 bL = cS.myPos().distanceRef(cS.ballPos());
  Vector2 gL = cS.myPos().distanceRef(fP.enemyGoalPos());
  if (leavingField(fP, cS)) {
    return refrainFromLeavingStrategy(fP, cS);

  } else if (!ballIsDetected(fP, cS)) {
    return slalowingBackwardsStrategy(fP, cS);

  } else if (ballIsCaught(fP, cS, bL)) {
    if (closeToShoot(fP, cS, gL)) {
      return shootStrategy(fP, cS);
    } else {
      return accelerateToGoalStrategy(fP, cS);
    }

  } else {
    if (targetInFrontOfRobot(fP, cS, bL)) {
      return goToBallStrategy(fP, cS);
    } else {
      return goToBallAvoidingBallStrategy(fP, cS, bL);
    }
  }
}

bool leavingField(FieldProperties fP, RobotState cS) {
  return (cS.myPos().x() < -fP.fieldWidth()/2) || (fP.fieldWidth()/2 < cS.myPos().x()) || (cS.myPos().y() < -fP.fieldLength()/2) || (fP.fieldLength()/2 < cS.myPos().y());
}

bool ballIsDetected(FieldProperties fP, RobotState cS) {
  return cS.ballPos() != fP.noneVect();
}

bool targetInFrontOfRobot(FieldProperties fP, RobotState cS, Vector2 tL) {
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

int getBallSidePositionFromRobot(FieldProperties fP, RobotState cS, Vector2 bL) {
  if (bL.x() < - fP.robotRadius()*1.5) {
    return -1;
  } else if (fP.robotRadius()*1.5 < bL.x()) {
    return 1;
  } else {
    return 0;
  }
}


Vector2 refrainFromLeavingStrategy(FieldProperties fP, RobotState cS) {
  int xDirection = 0;
  int yDirection = 0;

  if (cS.myPos().x() < -fP.fieldWidth()/2) {
    xDirection = 10;
  } else if (fP.fieldWidth()/2 < cS.myPos().x()) {
    xDirection = -10;
  }

  if (cS.myPos().y() < -fP.fieldLength()/2) {
    yDirection = 10;
  } else if (fP.fieldLength()/2 < cS.myPos().y()) {
    yDirection = -10;
  }

  return Vector2(
    cS.myPos().x()+xDirection,
    cS.myPos().y()+yDirection
  );
}

Vector2 goToBallStrategy(FieldProperties fP, RobotState cS) {
  return Vector2(
      cS.ballPos().x(),
      cS.ballPos().y() - fP.robotRadius()*1.5
    );
}

Vector2 goToBallAvoidingBallStrategy(FieldProperties fP, RobotState cS, Vector2 bL) {
  if (getSidePosition(fP, cS) == -1) {
    if (getBallSidePositionFromRobot(fP, cS, bL) == -1){
      return Vector2(
        cS.myPos().x(),
        cS.myPos().y() -10
      );

    } else if (bL.norm() < 2*fP.robotRadius()){
        return Vector2(
          cS.myPos().x() +10,
          cS.myPos().y()
        );

    } else {
      return Vector2(
        cS.myPos().x() +10,
        cS.myPos().y() -10
      );
    }

  } else {
    if (getBallSidePositionFromRobot(fP, cS, bL) == 1){
      return Vector2(
        cS.myPos().x(),
        cS.myPos().y() -10
      );

    } else if (bL.norm() < 2*fP.robotRadius()){
        return Vector2(
          cS.myPos().x() -10,
          cS.myPos().y()
        );
        
    } else {
      return Vector2(
        cS.myPos().x() -10,
        cS.myPos().y() -10
      );
    }
  }
}

Vector2 accelerateToGoalStrategy(FieldProperties fP, RobotState cS) {
  return Vector2(
      fP.enemyGoalPos().x(),
      fP.enemyGoalPos().y() - 15
    );
}

Vector2 slalowingBackwardsStrategy(FieldProperties fP, RobotState cS) {
  if (cS.myPos().y() < -70){
    if (cS.myPos().x() < -5) {
      return Vector2(
        cS.myPos().x() +10,
        cS.myPos().y()
      );
    } else if (5 < cS.myPos().x()) {
      return Vector2(
        cS.myPos().x() -10,
        cS.myPos().y()
      );
    } else {
      return Vector2(
        cS.myPos().x(),
        cS.myPos().y() +10
      );
    }

  } else if (50 < cS.myPos().y()) {
    return Vector2(
      cS.myPos().x() -20,
      cS.myPos().y() -10
    );

  } else {
    if (cS.myPos().x() < -fP.fieldWidth()/6){
      return Vector2(
        cS.myPos().x() +20,
        cS.myPos().y() -10
      );
    } else if (fP.fieldWidth()/6 < cS.myPos().x()){
      return Vector2(
        cS.myPos().x() -20,
        cS.myPos().y() -10
      );
    } else {
      //On ne change pas la direction du robot (ajouter une condition pour ne rien changer si noneVect est renvoyé)
      return fP.noneVect();
    }
  }
}

Vector2 shootStrategy(FieldProperties fP, RobotState cS) {
  //digitalWrite(KICKER, HIGH);
  //sleep(0.01);
  //digitalWrite(KICKER, LOW);
  return Vector2(0,20);
}