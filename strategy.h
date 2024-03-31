#ifndef STRATEGY_H
#define STRATEGY_H

#include <Arduino.h>

#include "states.h"
#include "utilities.h"

Vector2 chooseStrategy(FieldProperties fP, RobotState cS);

bool targetInFrontOfRobot(FieldProperties fP, RobotState cS, Vector2 tL);
bool targetJustInFrontOfRobot(FieldProperties fP, RobotState cS, Vector2 tL);
bool targetJustBehindOfRobot(FieldProperties fP, RobotState cS, Vector2 tL);
bool ballIsDetected(FieldProperties fP, RobotState cS, Vector2 tL);
bool ballIsCaught(FieldProperties fP, RobotState cS, Vector2 bL);
bool closeToShoot(FieldProperties fP, RobotState cS, Vector2 L);

int getSidePosition(FieldProperties fP, RobotState cS, Vector2 bL);

Vector2 goToBallStrategy(FieldProperties fP, RobotState cS, Vector2 bL);
Vector2 goToBallAvoidingBallStrategy(FieldProperties fP, RobotState cS, Vector2 bL);
Vector2 accelerateToGoalStrategy(FieldProperties fP, RobotState cS, Vector2 bL);
Vector2 slalowingBackwardsStrategy(FieldProperties fP, RobotState cS, Vector2 bL);
Vector2 shootStrategy(FieldProperties fP, RobotState cS, Vector2 bL);

#endif