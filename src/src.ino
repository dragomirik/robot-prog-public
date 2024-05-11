#include "lidar.h"
#include "lidar_analyzer.h"
#include "movements.h"
#include "states.h"
#include "strategy.h"
#include "utilities.h"

const FieldProperties fieldProperties = FieldProperties(
    243,               // fieldLength
    182,               // fieldWidth
    12,                // spaceBeforeLineSide
    60,                // goalWidth
    Vector2(0, -115),  // myGoalPos
    Vector2(0, 115),   // enemyGoalPos
    9,                 // robotRadius
    2                  // ballRadius
);

const Motors motors = Motors(

    // Arduino UNO
    // MotorMov(11, 12, 0, Degree(-40)),
    // MotorMov(5, 4, 0, Degree(40)),
    // MotorMov(6, 7, 0, Degree(-140)),
    // MotorMov(9, 8, 0, Degree(140))

    // Teensy
    MotorMov(3, 4, 0, Degree(-40)),
    MotorMov(11, 12, 0, Degree(40)),
    MotorMov(22, 23, 0, Degree(-140)),
    MotorMov(36, 37, 0, Degree(140)));

ReadingData readingData = ReadingData();

RobotState currentState = RobotState(
    Vector2(0, 0),
    Vector2(0, 0),
    Vector2(0, 0),
    Vector2(0, 0),
    Vector2(0, 0));

void setup() {
  SerialDebug.begin(115200);
  SerialCam.begin(115200);
  SerialLidar.begin(230400);
}

void loop() {
  // GETTING LIDAR DATA
  // TODO
  delay(100);

  // GETTING CAM DATA
  while (true) {
    if (SerialCam.available()) {
      char newChar = SerialCam.read();
      if (currentState.updateFromString(readingData, newChar)) {
        break;
      }
    } else {
      break;
    }
  }

  // DOING ACTION
  // TODO: must work without lidar data or without cam data
  FutureAction action = chooseStrategy(fieldProperties, currentState);
  if (action.changeMove()) {
    // TODO: FutureAction must include rotation and speed !
    motors.goTo(action.target(), 255, currentState.enemyGoalPos().angle());
  }
  if (action.activeKicker()) {
    // TODO active kicker
  }
}