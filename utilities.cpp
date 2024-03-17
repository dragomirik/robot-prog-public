#include "utilities.h"

///////VECTOR2

Vector2::Vector2(float x, float y)
    : _x(x), _y(y) {}

String Vector2::toString() const {
  return "(" + String(_x) + ", " + String(_y) + ")";
}

bool Vector2::operator==(const Vector2 &other) {
  return (_x == other._x && _y == other._y);
}
bool Vector2::operator!=(const Vector2 &other) {
  return (_x != other._x || _y != other._y);
}

Vector2 Vector2::distanceRef(Vector2 other) const {
  return Vector2(
      x() - other.x(),
      other.y() - y());
}

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