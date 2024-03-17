#ifndef UTILITIES_H
#define UTILITIES_H

#include <Arduino.h>

class Vector2 {
 public:
  Vector2(float x, float y);
  float x() const;
  float y() const;
  String toString() const;

  bool operator==(const Vector2 &other);
  bool operator!=(const Vector2 &other);
  Vector2 distanceRef(Vector2 other) const;

 private:
  const float _x, _y;
};

#endif