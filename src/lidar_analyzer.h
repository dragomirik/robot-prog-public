#ifndef LIDAR_ANALYZER
#define LIDAR_ANALYZER

#include <Arduino.h>
#include "utilities.h"
#include "strategy.h"

class RobotInfos {
  public:
    RobotInfos(const Vector2& coordinates, double orientation, std::vector<MutableVector2> walls)
        : coordinates(coordinates), orientation(orientation), walls(walls) {}

  /* Retourne les coordonnées du robot dans le référentiel du terrain. Centre du terrain: x=0, y=0. 
     Axe y positif dans la direction du regard du robot 
  */
  Vector2 getCoordinates() {
    return coordinates;
  }

  /* Retourne l'orientation du robot (en degrés) : 0° s'il regarde droit vers le goal, < 0 s'il regarde vers la gauche, > 0 s'il regarde vers la droite
     max 90° (ensuite tout le repère s'inverse) 
  */
  double getOrientation() {
    return orientation * (180.0 / M_PI);
  }

  /* retourne les murs (le point le plus proche de chaque mur) */
  std::vector<MutableVector2> getWalls() {
    return walls;
  }

  /* Retourne le point du mur le plus proche */
  MutableVector2 getNearestWall() {
    if (walls.empty()) {
      return MutableVector2({-9999, -9999});
    }

    float nearest = 100000;
    size_t indice;
    for (size_t i = 0; i < walls.size(); i++) {
      float distance = walls[i].toVector2().distance({0, 0});
      if (distance < nearest) {
        nearest = distance;
        indice = i;
      }
    }
    return walls[indice];
  }

  private: 
    Vector2 coordinates;
    double orientation;
    std::vector<MutableVector2> walls;
};

RobotInfos getFieldInfos(FieldProperties fP, bool readFromLidar = true, bool show_log = false, const char* input = nullptr);
void testsLidar(FieldProperties fP);

#endif