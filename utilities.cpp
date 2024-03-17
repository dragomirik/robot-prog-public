#include "utilities.h"

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