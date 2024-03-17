#ifndef UTILITIES_H
#define UTILITIES_H

#include <Arduino.h>

class Vector2 {
 public:
  Vector2(float x, float y);
  inline float x() const { return _x; };
  inline float y() const { return _y; };
  String toString() const;

  bool operator==(const Vector2 &other);
  bool operator!=(const Vector2 &other);
  Vector2 distanceRef(Vector2 other) const;

 private:
  const float _x, _y;
};

#endif