#ifndef LIDAR_ANALYZER
#define LIDAR_ANALYZER

#include <Arduino.h>
#include "utilities.h"
#include "strategy.h"

struct HoughLine {
    double rho;
    double theta;
    double nb_accumulators;
    double length;
};

struct Point {
    double x, y;
};

struct RobotInfos {
  Point coordinates;
  double orientation;
  double nearestWall_distance;

  // Retourne les coordonnées du robot dans le référentiel terrain. (0, 0) est le centre du terrain
  Vector2 getCoordinates() {
    return Vector2(coordinates.x, coordinates.y);
 }

  /* Retourne l'orientation du robot (en degrés) : 0° s'il regarde droit vers le goal, < 0 s'il regarde vers la gauche, > 0 s'il regarde vers la droite
     max 90° (ensuite tout le repère s'inverse) 
  */
  double getOrientation() {
    return orientation * (180.0 / M_PI);
  }
};

bool findIntersection(const HoughLine& l1, const HoughLine& l2, double& x, double& y);
Point computeCentroid(const std::vector<Vector2>& corners);
std::vector<HoughLine> houghTransform(const std::vector<Vector2>& points, int numRho, int numTheta, double thetaStep);
RobotInfos getFieldInfos(FieldProperties fP, bool readFromLidar = true, bool show_log = false, const char* input = nullptr);
void testsLidar(FieldProperties fP);

#endif