#include "lidar.h"
#include "movements.h"
#include "states.h"
#include "strategy.h"
#include "utilities.h"

//TODO: test ball detecte

const FieldProperties fieldProperties = FieldProperties(
    5,    // fieldLength
    10,   // fieldDepth
    0,    // spaceBeforeLineSide
    2,    // goalWidth
    Vector2(0,0),   //myGoalPos
    Vector2(0,0),   //enemyGoalPos
    0.2,  // robotRadius
    0.05  // ballRadius
);

const Motors motors = Motors(
    MotorMov(1, 2, 3, -55),
    MotorMov(4, 5, 6, 55),
    MotorMov(7, 8, 9, -125),
    MotorMov(10, 11, 12, 125));

CircularLidarPointsBuffer lidarPointsBuffer = CircularLidarPointsBuffer(200);

void setup() {
  SerialDebug.begin(230400);
  SerialCam.begin(115200);
  SerialLidar.begin(230400);
}

/*void loop() {
  RobotState currentState = RobotState::fromString(
    RobotState(Vector2(0, 0), Vector2(0, 0), Vector2(0, 0)),
    SerialCam.readStringUntil("", 100));

  SerialDebug.println(currentState.toString());
}*/
/*
void loop() {
  //BUG: Buffer doesn't work
  SerialDebug.println("----------");
  for (unsigned int i=0; i < 360; i++) {
    lidarPointsBuffer.addValue(LidarPoint(
    i*10,
    0,
    i*100
  ));
  }
  savePointsLocal(lidarPointsBuffer);
  lidarPointsBuffer.flush();
}*/

void loop() {
  motors.goTo(Vector2(0, 100), 255);
}