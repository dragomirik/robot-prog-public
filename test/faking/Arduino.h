#ifndef FAKE_ARDUINO_H
#define FAKE_ARDUINO_H

#include <math.h>

#include <cctype>
#include <queue>
#include <string>

#define PI 3.1415926535897932384626433832795
#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105

#define sqrt std::sqrt
#define isDigit std::isdigit

double sq(double x);

std::string _removeZeros(std::string str);
class String : public std::string {
 public:
  String();
  String(const char* s);
  String(const std::string& s);
  String(double s);

  float toFloat();
};

class SerialClass {
 private:
  std::queue<char> _incomingData;
  std::queue<char> _outgoingData;

 public:
  SerialClass();
  void begin(int baud);
  char read();
  void write(char data);
  bool available();
  void println(const std::string& str);
  char debugRead();
  void debugWrite(char data);
  bool debugAvailable();
  void debugPrintln(const std::string& str);
};

extern SerialClass Serial;
extern SerialClass Serial1;
extern SerialClass Serial2;
extern SerialClass Serial3;
extern SerialClass Serial4;
extern SerialClass Serial5;
extern SerialClass Serial6;
extern SerialClass Serial7;
extern SerialClass Serial8;

#endif