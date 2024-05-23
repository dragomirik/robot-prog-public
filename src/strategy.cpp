#include "strategy.h"

FutureAction::FutureAction(
    Vector2 target,
    int celerity,
    Radians orientation,
    bool activeKicker)
    : _target(Optional<MutableVector2>(target)),
      _celerity(celerity),
      _orientation(orientation),
      _activeKicker(activeKicker) {}

FutureAction::FutureAction(
    int celerity,
    Radians orientation,
    bool activeKicker)
    : _target(Optional<MutableVector2>()),
      _celerity(celerity),
      _orientation(orientation),
      _activeKicker(activeKicker) {}

////////
const int speedmotors = 100;
const int goalMinDistance = 75;
const FutureAction stopRobot = FutureAction(Vector2(0, 0), 0, 0, false);
FutureAction chooseStrategy(FieldProperties fP, RobotState cS, double orientation, Vector2 nearestWall) {
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
      return refrainFromLeavingStrategy(fP, cS, orientation, nearestWall);

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
  return cS.myPos() == Vector2(-9999, -9999);
}

bool leavingField(FieldProperties fP, RobotState cS) {
  SerialDebug.println("Left wall : " + String(cS.myPos().x() < -fP.fieldWidth() / 2 + 3*fP.robotRadius())
  + " Right wall : " + String((fP.fieldWidth() / 2) - 3*fP.robotRadius() < cS.myPos().x())
  + " Back wall : " + String(cS.myPos().y() < -fP.fieldLength() / 2 + 4*fP.robotRadius())
  + " Front wall : " + String(fP.fieldLength() / 2 - 4*fP.robotRadius() < cS.myPos().y())
  + " Enemy goal : " + String(cS.enemyGoalPos().norm() < goalMinDistance && cS.enemyGoalPos().norm() > 1)
  + " My goal : " + String(cS.myGoalPos().norm() < goalMinDistance && cS.myGoalPos().norm() > 1)  
  );

  // SerialDebug.println(cS.myPos().y());
  // SerialDebug.println(fP.fieldLength());
  
  return (cS.myPos().x() < -fP.fieldWidth() / 2 + 3*fP.robotRadius()) ||
         (fP.fieldWidth() / 2 - 3*fP.robotRadius() < cS.myPos().x()) ||
         (cS.myPos().y() < -fP.fieldLength() / 2 + 4*fP.robotRadius()) ||
         (fP.fieldLength() / 2 - 4*fP.robotRadius() < cS.myPos().y()) ||
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
  return abs(tL.x()) <= 6;
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

FutureAction refrainFromLeavingStrategy(FieldProperties fP, RobotState cS, double orientation, Vector2 nearestWall) {
  SerialDebug.println("refrainFromLeavingStrategy");
  int xDirection = 0;
  int yDirection = 0;

  if (cS.myPos().x() < -fP.fieldWidth() / 2 + 3*fP.robotRadius()) {
    xDirection = 10*cos(orientation);
    yDirection = 10*sin(orientation);
  } else if (fP.fieldWidth() / 2 - 3*fP.robotRadius() < cS.myPos().x()) {
    xDirection = -10*cos(orientation);
    yDirection = -10*sin(orientation);
  }

  if (cS.myPos().y() < -fP.fieldLength() / 2 + 4*fP.robotRadius() ||
      cS.enemyGoalPos().norm() < goalMinDistance) {
    xDirection = 10*sin(orientation);
    yDirection = 10*cos(orientation);
  } else if (fP.fieldLength() / 2 - 4*fP.robotRadius() < cS.myPos().y() ||
             cS.myGoalPos().norm() < goalMinDistance) {
    xDirection = -10*sin(orientation);
    yDirection = -10*cos(orientation);
  }

  if (nearestWall.distance({0,0}) / 10.0 < 4*fP.robotRadius()) {
    xDirection = -nearestWall.x();
    yDirection = -nearestWall.y();
  }

  SerialDebug.println(String(xDirection) + " " + String(yDirection));
  return FutureAction(
      Vector2(
          xDirection,
          yDirection),
      speedmotors,
      0,
      false); 
}

FutureAction goToBallStrategy(FieldProperties fP, RobotState cS) {
  SerialDebug.println("goToBallStrategy");
  return FutureAction(
      Vector2(
          cS.ballPos().x(),
          cS.ballPos().y() - fP.robotRadius() * 4),
      speedmotors,
      0,
      false);  
}

FutureAction goToBallAvoidingBallStrategyWithCam(FieldProperties fP, RobotState cS) {
  if (targetJustBehindOfRobot(fP, cS, cS.ballPos())) {
    return FutureAction(
        Vector2(10, -10),
        speedmotors,
        0,
        false);  

  } else if (cS.myPos().x() < 0) {
    return FutureAction(
        Vector2(2, -10),
        speedmotors,
        0,
        false);  
  } else if (cS.myPos().x() < 0) {
    return FutureAction(
        Vector2(-2, -10),
        speedmotors,
        0,
        false);  
  }
}

FutureAction goToBallAvoidingBallStrategyWithLidar(FieldProperties fP, RobotState cS) {
  SerialDebug.println("goToBallAvoidingBallStrategyWithLidar");
  if (targetJustBehindOfRobot(fP, cS, cS.ballPos())) {
    if (cS.myPos().x() < 0) {
      return FutureAction(
          Vector2(10, -10),
          speedmotors,
          0,
          false); 

    } else {
      return FutureAction(
          Vector2(-10, -10),
          speedmotors,
          0,
          false);  
    }

  } else if (cS.myPos().x() < 0) {
    return FutureAction(
        Vector2(2, -10),
        speedmotors,
        0,
        false);  
  } else if (cS.myPos().x() > 0) {
    return FutureAction(
        Vector2(-2, -10),
        speedmotors,
        0,
        false);  
  }
}

FutureAction accelerateToGoalStrategyWithCam(FieldProperties fP, RobotState cS) {
  SerialDebug.println("accelerateToGoalStrategyWithCam");
  
  int orientation;
  if (cS.enemyGoalPos().x() > 10) {
    orientation = 10;
  } else if (cS.enemyGoalPos().x() < -10) {
    orientation = -10;
  }
  
  return FutureAction(
      Vector2(
          cS.enemyGoalPos().x(),
          cS.enemyGoalPos().y() - 10),
      speedmotors,
      orientation,
      false); 
}

FutureAction accelerateToGoalStrategyWithLidar(FieldProperties fP, RobotState cS) {
  SerialDebug.println("accelerateToGoalStrategyWithLidar");

  int orientation = 0;
  if (cS.myPos().x() > 20) {
    orientation = -20;
  } else if (cS.myPos().x() > 20) {
    orientation = 20;
  }

  return FutureAction(
      Vector2(
          fP.enemyGoalPos().x() - cS.myPos().x(),
          fP.enemyGoalPos().y() - cS.myPos().y() - 10),
      speedmotors,
      orientation,
      false);  
}

FutureAction slalowingBackwardsStrategy(FieldProperties fP, RobotState cS) {
  SerialDebug.println("slalowingBackwardsStrategy");
  if (cS.myPos().y() < -50) {
    return FutureAction(
           Vector2(0, 0),
           0,
           0,
           false);  
    // if (cS.myPos().x() < -5) {
    //   return FutureAction(
    //       Vector2(10, 0),
    //       speedmotors,
    //       0,
    //       false);  
    // } else if (5 < cS.myPos().x()) {
    //   return FutureAction(
    //       Vector2(-10, 0),
    //       speedmotors,
    //       0,
    //       false);  
    // } else {
    //   return FutureAction(
    //       Vector2(0, 10),
    //       speedmotors,
    //       0,
    //       false);  
    // }

  } else if (50 < cS.myPos().y()) {
    return FutureAction(
        Vector2(-20, -10),
        speedmotors,
        0,
        false); 

  } else {
    if (cS.myPos().x() < -fP.fieldWidth() / 6) {
      return FutureAction(
          Vector2(20, -10),
          speedmotors,
          0,
          false);  
    } else if (fP.fieldWidth() / 6 < cS.myPos().x()) {
      return FutureAction(
          Vector2(-20, -10),
          speedmotors,
          0,
          false);  
    } else {
      return FutureAction(
          speedmotors,
          0,
          false); 
    }
  }
}

FutureAction shootStrategy(FieldProperties fP, RobotState cS) {
  SerialDebug.println("shootStrategy");
  
  int shootSpeed;
  if (speedmotors*2 > 255) {
    shootSpeed = 255;
  } else {
    shootSpeed = speedmotors*2;
  }
  
  return FutureAction(
      Vector2(0, 20),
      shootSpeed,
      0,
      true);  
}
