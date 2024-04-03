#ifndef LIDAR_ANALYZER
#define LIDAR_ANALYZER

#include <Arduino.h>
#include "utilities.h"

/*
struct Line {
    double rho;
    double theta;
    double nb_accumulators;
};

struct Vector3 {
    double x, y;
};

struct FieldInfos {
  Vector3 center;
  float orientation;
};

bool findIntersection(const Line& l1, const Line& l2, double& x, double& y);
Vector3 computeCentroid(const std::vector<Vector2>& corners);
std::vector<Line> houghTransform(const std::vector<Vector2>& points, int numRho, int numTheta, double thetaStep, int threshold);
FieldInfos getFieldInfos(bool readFromLidar = true, bool show_log = false);
*/
#endif