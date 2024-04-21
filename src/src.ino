#include "lidar.h"
#include "movements.h"
#include "states.h"
#include "strategy.h"
#include "lidar_analyzer.h"
#include "utilities.h"

const FieldProperties fieldProperties = FieldProperties(
    219,               // fieldLength
    158,               // fieldWidth
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

/*std::vector<Line> houghTransform(const std::vector<Vector2>& points, int numRho, double rhoStep, int numTheta, double thetaStep, int threshold) {
    std::vector<Line> lines;
    std::vector<int> accumulator(numRho * 2 * numTheta / rhoStep, 0);
    SerialDebug.println("accumulator: " + String(accumulator.size()));

    for (const auto& point : points) {
      SerialDebug.println("point : x=" + String(point.x()) + ", y=" + String(point.y()));
        for (int thetaIndex = 0; thetaIndex < numTheta; thetaIndex++) {
            double theta = thetaIndex * thetaStep;
            int rhoIndex = round(numRho + (point.x() * cos(theta) + point.y() * sin(theta) / rhoStep));
            int accuIndex = thetaIndex * numRho + rhoIndex;
            // SerialDebug.println("theta=" + String(theta) + ", rhoIndex=" + String(rhoIndex)
                // + ", accu index=" + String(accuIndex));
            if (accuIndex >= 0 && accuIndex < accumulator.size()) {
                accumulator[accuIndex]++;
            }
        }
    }

    int max_threshold = 0;
    for (int thetaIndex = 0; thetaIndex < numTheta; thetaIndex++) {
        for (int rhoIndex = 0; rhoIndex < numRho * 2; rhoIndex++) {
          // SerialDebug.println("accu(" + String(thetaIndex) + "," + String(rhoIndex) +
                  // ")=" + String(accumulator[thetaIndex * numRho + rhoIndex]));
          if(accumulator[thetaIndex * numRho + rhoIndex] > max_threshold) {
            max_threshold = accumulator[thetaIndex * numRho + rhoIndex];
          }

          if (accumulator[thetaIndex * numRho + rhoIndex] >= threshold) {
              Line line;
              line.accumulator_index = thetaIndex * numRho + rhoIndex;
              line.rho = rhoIndex * rhoStep - numRho;
              line.theta = thetaIndex * thetaStep;
              lines.push_back(line);
          }
        }
    }
    SerialDebug.println("max threshold=" + String(max_threshold));

    return lines;
}*/

int counter = 0;

void loop() {
  counter++;
  RobotInfos infos;
  double nearestWallDistance = 0;

  // testsLidar(fieldProperties); // tests du LIDAR
  
  if(fmod(counter, 30) == 0) {
    counter = 0;
    infos = getFieldInfos(fieldProperties, true, true);
    if(infos.nearestWall_distance > 0) {
      nearestWallDistance = infos.nearestWall_distance;
    }
    SerialDebug.println("Coordonnées robot: x=" + String(infos.coordinates.x / 10.0) + " cm, y=" + String(infos.coordinates.y / 10.0)
      + " cm, orientation: " + String(infos.getOrientation()) + "°, Nearest Wall distance=" 
      + String(infos.nearestWall_distance / 10.0) + " cm");
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

    /*
    unsigned long start_millis = millis();

    CircularLidarPointsBuffer lidarPointsBuffer = CircularLidarPointsBuffer(nbLidarPoints);
    std::vector<LidarPoint> points;

    const int nb_tours_lidar = 55;
    double distance_max = 0;

    for (int i = 0; i < nb_tours_lidar; i++) {
        std::vector<LidarPoint> lidarPoints = lidarPointsBuffer.getPoints();
        for (size_t j = 0; j < lidarPoints.size(); j++) {
          LidarPoint lidarPoint = lidarPoints[j];
          if(lidarPoint.distance() > 100) { // on ne prend pas les points < 10cm
            points.push_back(lidarPoint);
            if(lidarPoint.distance() > distance_max) {
              distance_max = lidarPoint.distance();
            }
          }
        }
    }

    unsigned long elapsed = millis() - start_millis;
    SerialDebug.println("Temps d'exécution : " + String(elapsed) + "ms");
    SerialDebug.println("First lidarPoint(0): " + points[0].toString());
    SerialDebug.println("Last lidarPoint(" + String(points.size()) + "): " + points[points.size() - 1].toString());

    // SerialDebug.println("*** log pour visualisation ***");
    String log = "";
    for (size_t i = 0; i < points.size(); i++) {
      LidarPoint lidarPoint = points[i];
      log += "(" + String(lidarPoint.angle()) + "," + String(lidarPoint.distance()) + ")," ;
    }
    SerialDebug.println(log);

    // coordonnées cartésiennes :
    std::vector<Vector2> points_cart;
    double sumX = 0.0, sumY = 0.0;
    for (size_t i = 0; i < points.size(); i++) {
      LidarPoint lidarPoint = points[i];
      float x = lidarPoint.distance() * std::cos(lidarPoint.angle() / 18000.0 * M_PI);
      float y = lidarPoint.distance() * std::sin(lidarPoint.angle() / 18000.0 * M_PI);
      points_cart.push_back(Vector2(x, y));
      sumX += x;
      sumY += y;
    }

    // SerialDebug.println("*** points en coordonnées cartésiennes ***");
    // String log2 = "";
    // for (size_t i = 0; i < points_cart.size(); i++) {
    //   Vector2 point = points_cart[i];
    //   log2 += "(" + String(point.x()) + "," + String(point.y()) + ")," ;
    // }
    // SerialDebug.println(log2);

    double pos_x = sumX / points.size();
    double pos_y = sumY / points.size();
    SerialDebug.println("position: x=" + String(pos_x / 10.0) + "cm, y=" + String(pos_y / 10.0) + "cm");

    SerialDebug.println("distance_max: " + String(distance_max));


    // int numRho = distance_max;
    double rhoStep = 0.1;
    int numTheta = 180;
    double thetaStep = 0.1 * M_PI / 180.0;
    int threshold = 5;

    // Appliquer la transformation de Hough
    int numRho = 4;
    std::vector<Vector2> points_cart;
    // points_cart.clear();
    points_cart.push_back(Vector2(0, 4));
    points_cart.push_back(Vector2(1, 3));
    points_cart.push_back(Vector2(2, 2));
    points_cart.push_back(Vector2(3, 1));
    points_cart.push_back(Vector2(4, 0));

    std::vector<Line> lines = houghTransform(points_cart, numRho, rhoStep, numTheta, thetaStep, threshold);
    SerialDebug.println("lines: " + String(lines.size()));
    // Afficher les lignes détectées
    for (const auto& line : lines) {
        SerialDebug.println("accu index: " + String(line.accumulator_index)
            + ", rho: " + String(line.rho) + ", theta: " + String(line.theta));
    }

    SerialDebug.println("pause");
    delay(2000);
  */
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
