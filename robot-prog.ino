#include "lidar.h"
#include "movements.h"
#include "states.h"
#include "strategy.h"
#include "utilities.h"

const FieldProperties fieldProperties = FieldProperties(
    5,   // fieldLength
    10,  // fieldDepth
    0,   // spaceBeforeLineSide
    2    // goalWidth
);

const Motors motors = Motors(
    MotorMov(1, 2, 3, -55),
    MotorMov(4, 5, 6, 55),
    MotorMov(7, 8, 9, -125),
    MotorMov(10, 11, 12, 125));

void setup() {
  SerialDebug.begin(230400);
  SerialCam.begin(115200);
  SerialLidar.begin(230400);
}

void loop() {
  RobotState currentState = RobotState::fromString(
    RobotState(Vector2(0, 0), Vector2(0, 0), Vector2(0, 0)),
    SerialCam.readStringUntil("", 100));

  SerialDebug.println(currentState.toString());
}