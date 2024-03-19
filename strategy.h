#ifndef STRATEGY_H
#define STRATEGY_H

#include <Arduino.h>
#include "states.h"

Vector2 chooseStrategy(FieldProperties fieldProperties, RobotState currentState);

Vector2 goToBallStrategy(FieldProperties fieldProperties, RobotState currentState);
Vector2 goToBallAvoidingBallStrategy(FieldProperties fieldProperties, RobotState currentState);
Vector2 accelerateToGoalStrategy(FieldProperties fieldProperties, RobotState currentState);
Vector2 slalowingBackwardsStrategy(FieldProperties fieldProperties, RobotState currentState);
Vector2 shootStrategy(FieldProperties fieldProperties, RobotState currentState);

#endif