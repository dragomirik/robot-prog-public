#include "strategy.h"


Vector2 chooseStrategy(FieldProperties fP, RobotState cS) {
  if (!ballIsDetected(fP, cS)) {
    return slalowingBackwardsStrategy(fP, cS);
  } else if (ballIsCaught(fP, cS)) {
    if (closeToShoot(fP, cS)) {
      return shootStrategy(fP, cS);
    } else {
      return accelerateToGoalStrategy(fP, cS);
    }
    accelerateToGoalStrategy(fP, cS);
  } else {
    if (ballInFrontOfRobot(fP, cS)) {
      return goToBallStrategy(fP, cS);
    } else {
      return goToBallAvoidingBallStrategy(fP, cS);
    }
  }
}