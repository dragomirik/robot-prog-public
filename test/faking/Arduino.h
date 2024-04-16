#pragma once

#include <string>
#include <math.h>

#define PI 3.1415926535897932384626433832795
#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105

#define sqrt std::sqrt

double sq(double x);

class String : public std::string {
  public:
    String();
    String(const char* s);
    String(const std::string& s);
    String(double s);
};