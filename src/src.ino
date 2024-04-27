#include "lidar.h"
#include "movements.h"
#include "states.h"
#include "strategy.h"
#include "lidar_analyzer.h"
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
    MotorMov(36, 37, 0, Degree(140))
);

size_t savedIndex = 0;

ReadingData readingData = ReadingData();

int nbLidarPoints = 456;

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

int counter = 0;

void loop() {
  counter++;
  double nearestWallDistance = 0;

  //testsLidar(fieldProperties); // tests du LIDAR
  // RobotInfos infos = getFieldInfos(fieldProperties, true, true);
  // SerialDebug.println("Coordonnées robot: x=" + String(infos.getCoordinates().x() / 10.0) + " cm, y=" + String(infos.getCoordinates().y() / 10.0)
      // + " cm, orientation: " + String(infos.getOrientation()) + "°, Nearest Wall distance=" 
      // + String(infos.getNearestWall().toVector2().distance({0,0}) / 10.0) + " cm");

  //delay(1000);
  //return;
  
  if(fmod(counter, 30) == 0) {
    counter = 0;
    RobotInfos infos = getFieldInfos(fieldProperties, true, true, nullptr);
    SerialDebug.println("Coordonnées robot: x=" + String(infos.getCoordinates().x() / 10.0) + " cm, y=" + String(infos.getCoordinates().y() / 10.0)
      + " cm, orientation: " + String(infos.getOrientation()) + "°, Nearest Wall distance=" 
      + String(infos.getNearestWall().toVector2().distance({0,0}) / 10.0) + " cm");
  }

  if (SerialCam.available()) {
    char newChar = SerialCam.read();
    // SerialDebug.println('"' + String(newChar) + '"');
    if (currentState.updateFromString(readingData, newChar)) {
      currentState.nearestWallDistance = nearestWallDistance;
      int speedMotors = 120;
      FutureAction action = chooseStrategy(fieldProperties, currentState);
      /*
      if (target == Vector2(fieldProperties.enemyGoalPos().x(), fieldProperties.enemyGoalPos().y() + 15)) {
        speedMotors = 255;
      }*/
      if (action.changeMove()) {
        motors.goTo(action.target(), speedMotors, currentState.enemyGoalPos().angle());
      }
      if (action.activeKicker()) {
        //active kicker
      }
    }
  }
}
/*
  while (SerialCam.available()) {
    //CAM
    char newChar = SerialCam.read();
    SerialDebug.println('"' + String(newChar) + '"');
    if (currentState.updateFromString(typeState, xReadingState, yReadingState, writingInXState, newChar)) {

      float myGoalAngle = currentState.myGoalPos().angle();
      if (myGoalAngle >= 0) {
        myGoalAngle -= 180;
      } else {
        myGoalAngle += 180;
      }
      float orientationRobotCam = (currentState.enemyGoalPos().angle() + myGoalAngle)/2;

      float angleBalleRefRobot = currentState.ballPos().angle() + orientationRobotCam;
      float distanceBalleRefRobot = 0.25 * pow(2.718281828459045, 0.054 * currentState.ballPos().norm()) + 5;
      Vector2 positionBalleRefRobot(distanceBalleRefRobot * cos(angleBalleRefRobot), distanceBalleRefRobot * cos(angleBalleRefRobot));
      Vector2 positionBalleRefTerrain(currentState.myPos().x() + positionBalleRefRobot.x(), currentState.myPos().y() + positionBalleRefRobot.y());
    }
    //LIDAR
    //lidarPointsBuffer.readPointsAndAddToBuffer();

    //TODO detection murs


    //STRATEGY
    Vector2 target = chooseStrategy(fieldProperties, currentState);
    motors.goTo(target, 255);
  }
}*/
