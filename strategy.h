#ifndef STRATEGY_H
#define STRATEGY_H

#include <Arduino.h>
#include "utilities.h"
#include "states.h"

Vector2 chooseStrategy(FieldProperties fP, RobotState cS);

bool ballInFrontOfRobot(FieldProperties fP, RobotState cS);
bool ballIsDetected(FieldProperties fP, RobotState cS);
bool ballIsCaught(FieldProperties fP, RobotState cS);
bool closeToShoot(FieldProperties fP, RobotState cS);

Vector2 goToBallStrategy(FieldProperties fP, RobotState cS);
Vector2 goToBallAvoidingBallStrategy(FieldProperties fP, RobotState cS);
Vector2 accelerateToGoalStrategy(FieldProperties fP, RobotState cS);
Vector2 slalowingBackwardsStrategy(FieldProperties fP, RobotState cS);
Vector2 shootStrategy(FieldProperties fP, RobotState cS);

#endif