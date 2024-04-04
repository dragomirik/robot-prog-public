#ifndef LIDAR_ANALYZER
#define LIDAR_ANALYZER

#include <Arduino.h>
#include "utilities.h"

struct Line {
    double rho;
    double theta;
    double nb_accumulators;
};

struct Vector3 {
    double x, y;
};

struct RobotInfos {
  Vector3 coordinates;
  float orientation;
  double frontWall_distance;
  double nearestWall_distance;
};

bool findIntersection(const Line& l1, const Line& l2, double& x, double& y);
Vector3 computeCentroid(const std::vector<Vector2>& corners);
std::vector<Line> houghTransform(const std::vector<Vector2>& points, int numRho, int numTheta, double thetaStep, int threshold);
RobotInfos getFieldInfos(bool readFromLidar = true, bool show_log = false);

#endif