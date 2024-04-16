#pragma once

#include <string>
#include <math.h>

#define PI 3.1415926535897932384626433832795
#define sqrt std::sqrt

double sq(double x);

class String : public std::string {
  public:
    String();
    String(const char* s);
    String(const std::string& s);
    String(double s);
};