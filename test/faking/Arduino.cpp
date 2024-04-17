#include "Arduino.h"

SerialClass Serial;
SerialClass Serial1;
SerialClass Serial2;
SerialClass Serial3;
SerialClass Serial4;
SerialClass Serial5;
SerialClass Serial6;
SerialClass Serial7;
SerialClass Serial8;

double sq(double x) {
  return x * x;
}

std::string _removeZeros(std::string str) {
  size_t dotPos = str.find_last_of('.');
  if (dotPos != std::string::npos) {
    size_t nonZeroPos = str.find_last_not_of('0');
    if (nonZeroPos != std::string::npos && nonZeroPos > dotPos) {
      return str.substr(0, nonZeroPos + 1);
    } else if (nonZeroPos != std::string::npos && nonZeroPos == dotPos) {
      return str.substr(0, nonZeroPos + 2);
    }
  }
  return str;
}

String::String() : std::string() {}
String::String(const char* s) : std::string(s) {}
String::String(const std::string& s) : std::string(s) {}
String::String(double s) : std::string(_removeZeros(std::to_string(s))) {}

float String::toFloat() {
  return std::stof(*this);
}

SerialClass::SerialClass() {}

void SerialClass::begin(int baud) {}

char SerialClass::read() {
  if (!_incomingData.empty()) {
    char data = _incomingData.front();
    _incomingData.pop();
    return data;
  } else {
    return -1;
  }
}

void SerialClass::write(char data) {
  _outgoingData.push(data);
}

bool SerialClass::available() {
  return !_incomingData.empty();
}

bool SerialClass::debugAvailable() {
  return !_outgoingData.empty();
}

char SerialClass::debugRead() {
  if (!_outgoingData.empty()) {
    char data = _outgoingData.front();
    _outgoingData.pop();
    return data;
  } else {
    return -1;
  }
}

void SerialClass::debugWrite(char data) {
  _incomingData.push(data);
}

void SerialClass::println(const std::string& str) {
  for (char c : str) {
    write(c);
  }
  write('\n');
}

void SerialClass::debugPrintln(const std::string& str) {
  for (char c : str) {
    debugWrite(c);
  }
  debugWrite('\n');
}