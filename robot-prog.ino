#include "lidar.h"
#include "movements.h"
#include "states.h"
#include "strategy.h"
#include "utilities.h"

// TODO: test ball detecte

const FieldProperties fieldProperties = FieldProperties(
    5,              // fieldLength
    10,             // fieldDepth
    0,              // spaceBeforeLineSide
    2,              // goalWidth
    Vector2(0, 0),  // myGoalPos
    Vector2(0, 0),  // enemyGoalPos
    0.2,            // robotRadius
    0.05            // ballRadius
);

const Motors motors = Motors(

    // Arduino UNO
    MotorMov(11, 12, 0, Degree(-55)),
    MotorMov(5, 4, 0, Degree(55)),
    MotorMov(6, 7, 0, Degree(-125)),
    MotorMov(9, 8, 0, Degree(125))

    // Teensy
    /*MotorMov(25, 24, 0, Degree(-55)),
    MotorMov(3, 2, 0, Degree(55)),
    MotorMov(5, 4, 0, Degree(-125)),
    MotorMov(9, 6, 0, Degree(125))*/
);

size_t savedIndex = 0;
CircularLidarPointsBuffer lidarPointsBuffer = CircularLidarPointsBuffer(1000);

char typeState = 'x';
String xReadingState = "";
String yReadingState = "";
bool writingInXState = true;

RobotState currentState = RobotState(
    Vector2(0, 0),
    Vector2(0, 0),
    Vector2(0, 0));

void setup() {
  SerialDebug.begin(115200);
  // SerialCam.begin(115200);
  // SerialLidar.begin(230400);
}

/*
void loop() {
  while (SerialCam.available()) {
    char newChar = SerialCam.read();
    SerialDebug.println('"' + String(newChar) + '"');
    if (currentState.updateFromString(typeState, xReadingState, yReadingState, writingInXState, newChar)) {
      break;
    }
  }

  SerialDebug.println(currentState.toString());
}*/

void loop() {
  lidarPointsBuffer.readPointsAndAddToBuffer();
  savedIndex = lidarPointsBuffer.savePointsLocal(savedIndex);
}

/*
void loop() {
  motors.goTo(Vector2(100, 100), 50);
  delay(2000);
  motors.goTo(Vector2(-100, -100), 50);
  delay(2000);
  motors.goTo(Vector2(100, -100), 50);
  delay(2000);
  motors.goTo(Vector2(-100, 100), 50);
  delay(2000);
}*/