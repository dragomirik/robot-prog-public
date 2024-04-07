#ifndef UTILITIES_H
#define UTILITIES_H

#include <Arduino.h>

#define SerialDebug Serial
#define SerialCam Serial8
#define SerialLidar Serial2

class Radians;
class Degree;

class Vector2 {
 public:
  Vector2(float x, float y);
  inline float x() const { return _x; };
  inline float y() const { return _y; };
  String toString() const;

  inline bool operator==(const Vector2 &other) {
    return (_x == other._x && _y == other._y);
  }
  inline bool operator!=(const Vector2 &other) {
    return (_x != other._x || _y != other._y);
  }

  Vector2 distanceRef(Vector2 other) const;
  float distance(Vector2 other) const;
  float norm() const;
  float realNorm() const;
  Radians angle() const;
  Vector2 rotate(Radians rad);
  Vector2 transformToUV(Vector2 origin, Radians rotationAngle);

 private:
  const float _x, _y;
};

class MutableVector2 {
 public:
  MutableVector2(Vector2 vector2);

  inline float x() const { return _x; }
  inline float y() const { return _y; }

  String toString() const;
  Vector2 toVector2() const;

  inline bool operator==(const Vector2 &other) {
    return (_x == other.x() && _y == other.y());
  }
  inline bool operator!=(const Vector2 &other) {
    return (_x != other.x() || _y != other.y());
  }
  inline bool operator==(const MutableVector2 &other) {
    return (_x == other._x && _y == other._y);
  }
  inline bool operator!=(const MutableVector2 &other) {
    return (_x != other._x || _y != other._y);
  }

 private:
  float _x, _y;
};

class Degree {
 private:
  float _angle;

 public:
  Degree(float angle);
  Degree(Radians angle);
  inline operator float() const { return _angle; }
};

class Radians {
 private:
  float _angle;

 public:
  Radians(float angle);
  Radians(Degree angle);
  inline operator float() const { return _angle; }
};

#endif