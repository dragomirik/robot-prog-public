#include "lidar_analyzer.h"



HoughLine::HoughLine(const double rho, const double theta, const double nb_accumulators, const double length)
  :rho(rho), theta(theta), nb_accumulators(nb_accumulators), length(length) {}

RobotPosition::RobotPosition(const Vector2 coordinates, Radians orientation, std::vector<MutableVector2> walls)
      : coordinates(coordinates), orientation(orientation), walls(walls) {}
  