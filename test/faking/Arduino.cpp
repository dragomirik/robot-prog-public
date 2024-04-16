#include "Arduino.h"

double sq(double x) {
  return x * x;
}

std::string String(double value) {
  return std::to_string(value);
}


String::String() : std::string() {}
String::String(const char* s) : std::string(s) {}
String::String(const std::string& s) : std::string(s) {}
String::String(double s) : std::string(std::to_string(s)) {}
