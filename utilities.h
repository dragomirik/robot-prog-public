#ifndef UTILITIES_H
#define UTILITIES_H

#include <Arduino.h>

#define SerialDebug Serial
#define SerialCam Serial1

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

class Vector2OrError {
public:
  Vector2OrError(String message);
  Vector2OrError(Vector2 instance);

  inline bool isError() const { return _hasError; }
  inline Vector2 getVector2() const { return _instanceVector2; }
  inline String errorMessage() const { return _errorMessage; }

  Vector2 defaultIfError(Vector2 defaultVal) const;

private:
  const String _errorMessage;
  const Vector2 _instanceVector2;
  const bool _hasError;
};

#endif