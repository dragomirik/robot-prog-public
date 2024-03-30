#include "utilities.h"

///////VECTOR2

Vector2::Vector2(float x, float y)
    : _x(x), _y(y) {}

String Vector2::toString() const {
  return "(" + String(_x) + ", " + String(_y) + ")";
}

Vector2 Vector2::distanceRef(Vector2 other) const {
  return Vector2(
      other.x() - x(),
      other.y() - y());
}

float Vector2::distance(Vector2 other) const {
  return sqrt(sq(other.x() - x()) + sq(other.y() - y()));
}

///////VECTOR2ORERROR

Vector2OrError::Vector2OrError(String message)
    : _errorMessage(message),
      _instanceVector2(Vector2(-1, -1)),
      _hasError(true) {
  SerialDebug.println("Warning, unraised error :" + message);
}

Vector2OrError::Vector2OrError(Vector2 instance)
    : _errorMessage(""),
      _instanceVector2(instance),
      _hasError(false){};

Vector2 Vector2OrError::defaultIfError(Vector2 defaultVal) const {
  if (isError()) {
    return defaultVal;
  } else {
    return getVector2();
  }
}

///////MUTABLEVECTOR2

MutableVector2::MutableVector2(Vector2 vector2)
    : _x(vector2.x()), _y(vector2.y()) {}

String MutableVector2::toString() const {
  return "(Mutable," + String(_x) + ", " + String(_y) + ")";
}
Vector2 MutableVector2::toVector2() const {
  return Vector2(x(), y());
}

int sign(float n) {
  if (n >= 0) {
    return 1;
  } else {
    return -1;
  }
}