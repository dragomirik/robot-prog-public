#include "lidar.h"
#include "movements.h"
#include "states.h"
#include "strategy.h"
#include "utilities.h"

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

char typeState = 'x';
String xReadingState = "";
String yReadingState = "";
bool writingInXState = true;
int nbLidarPoints = 456;

RobotState currentState = RobotState(
    Vector2(0, 0),
    Vector2(0, 0),
    Vector2(0, 0));

void setup() {
  SerialDebug.begin(115200);
  // SerialCam.begin(115200);
  SerialLidar.begin(230400);
}

void loop() {
  CircularLidarPointsBuffer lidarPointsBuffer = CircularLidarPointsBuffer(nbLidarPoints);
  std::vector<LidarPoint> points;

  for (int i = 0; i < 50; i++) {
      std::vector<LidarPoint> lidarPoints = lidarPointsBuffer.getPoints();
      for (size_t j = 0; j < lidarPoints.size(); j++) {
        LidarPoint lidarPoint = lidarPoints[j];
        if(lidarPoint.distance() > 100) { // on ne prend pas les points < 10cm
          points.push_back(lidarPoint);
        }
      }
  }
  SerialDebug.println("First lidarPoint(0): " + points[0].toString());
  SerialDebug.println("Last lidarPoint(" + String(points.size()) + "): " + points[points.size() - 1].toString());

  // création du log pour visualisation : 
  String log = "";
  for (size_t i = 0; i < points.size(); i++) {
    LidarPoint lidarPoint = points[i];
    log += "(" + String(lidarPoint.angle()) + "," + String(lidarPoint.distance()) + ")," ;
  }
  SerialDebug.println(log);

  // calcul de la position :
  double sumX = 0.0, sumY = 0.0;
  for (size_t i = 0; i < points.size(); i++) {
    LidarPoint lidarPoint = points[i];
    float x = lidarPoint.distance() * std::cos(lidarPoint.angle() / 18000.0 * M_PI);
    float y = lidarPoint.distance() * std::sin(lidarPoint.angle() / 18000.0 * M_PI);
    sumX += x;
    sumY += y;
  }

  double pos_x = sumX / points.size();
  double pos_y = sumY / points.size();
  SerialDebug.println("position: x=" + String(pos_x / 10.0) + "cm, y=" + String(pos_y / 10.0) + "cm");

  SerialDebug.println("pause");
  delay(2000);
 

  /*while (SerialCam.available()) {
    //CAM
    char newChar = SerialCam.read();
    SerialDebug.println('"' + String(newChar) + '"');
    if (currentState.updateFromString(typeState, xReadingState, yReadingState, writingInXState, newChar)) {
      break;
    }
    //LIDAR
    lidarPointsBuffer.readPointsAndAddToBuffer();

    //TODO detection murs
    

    //STRATEGY
    Vector2 target = chooseStrategy(fieldProperties, currentState);
    motors.goTo(target, 255);
  }*/
}