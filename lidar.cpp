#include "lidar.h"

//////LIDARPOINT

LidarPoint::LidarPoint(uint16_t distance, uint8_t intensity)
    : _distance(distance), _intensity(intensity) {}

uint16_t LidarPoint::getStep(uint16_t startAngle, uint16_t endAngle, unsigned int lenMinusOne) {
  return (endAngle - startAngle) / lenMinusOne;
}

uint16_t LidarPoint::getAngle(uint16_t startAngle, uint16_t step, unsigned int indice) {
  return startAngle + (step * indice);
}

//////MUTABLELIDARPOINT

MutableLidarPoint::MutableLidarPoint(LidarPoint lidarPoint)
    : _distance(lidarPoint.distance()), _intensity(lidarPoint.intensity()) {}
MutableLidarPoint::MutableLidarPoint()
    : _distance(0), _intensity(0) {}

LidarPoint MutableLidarPoint::toLidarPoint() const {
  return LidarPoint(distance(), intensity());
}

uint16_t MutableLidarPoint::getStep(uint16_t startAngle, uint16_t endAngle, unsigned int lenMinusOne) {
  return (endAngle - startAngle) / lenMinusOne;
}

uint16_t MutableLidarPoint::getAngle(uint16_t startAngle, uint16_t step, unsigned int indice) {
  return startAngle + (step * indice);
}

//////CIRCULARLIDARPOINTSBUFFER

CircularLidarPointsBuffer::CircularLidarPointsBuffer(int bufferSize)
    : _size(bufferSize) {
  _buffer = new MutableLidarPoint[bufferSize];
}

CircularLidarPointsBuffer::~CircularLidarPointsBuffer() {
  delete[] _buffer;
}

void CircularLidarPointsBuffer::addValue(const LidarPoint newValue) {
  _buffer[_index] = MutableLidarPoint(newValue);
  if (_firstRound) {
    if (_index == _size - 1) {
      _firstRound = false;
    }
  }
  _index = (_index + 1) % _size;
}

bool CircularLidarPointsBuffer::existValue(size_t indice) const {
  if (_firstRound) {
    return indice < _index && indice >= 0;
  } else {
    return indice < _size && indice >= 0;
  }
}

LidarPoint CircularLidarPointsBuffer::getValue(size_t indice) const {
  // Be sure to check that the value exists with `existValue` before requesting it.
  return _buffer[(indice) % _size].toLidarPoint();
}

size_t CircularLidarPointsBuffer::sizeFilled() const {
  if (_firstRound) {
    return _index;
  } else {
    return _size;
  }
}

void CircularLidarPointsBuffer::flush() {
  _index = 0;
  _firstRound = true;
  delete[] _buffer;
  _buffer = new MutableLidarPoint[_size];
}

//////FUNCTIONS

uint8_t _calCRC8FromBuffer(uint8_t *p, uint8_t lenWithoutCRCCheckValue) {
  uint8_t crc = 0xD8;                                       //pre-calculated header and verlen values (crc = crcTable[(crc ^ 0x54) & 0xff];crc = crcTable[(crc ^ 0x2C) & 0xff];)
  for (uint16_t i = 0; i < lenWithoutCRCCheckValue; i++) {  //ignores the last value of the p array (which contains the crc check value)
    crc = crcTable[(crc ^ *p++) & 0xff];
  }
  return crc;
}

uint16_t _get2BytesLsbMsb(byte buffer[], int index) {
  return (buffer[index + 1] << 8) | buffer[index];
}
