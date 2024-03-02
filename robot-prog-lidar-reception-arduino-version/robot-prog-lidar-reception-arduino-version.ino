#define SerialDebug Serial
#define SerialLidar Serial2

///////// GLOBAL CLASSES
class Vector2 {  //NORMALE VERSION
private:
  const float _x, _y;
public:
  Vector2(float x, float y)
    : _x(x), _y(y) {}

  float x() const {
    return _x;
  }
  float y() const {
    return _y;
  }
  String toString() const {
    return "(" + String(_x) + ", " + String(_y) + ")";
  }

  bool operator==(const Vector2 &other) {
    return (_x == other._x && _y == other._y);
  }
  bool operator!=(const Vector2 &other) {
    return (_x != other._x || _y != other._y);
  }
};

class MutableVector2 {
private:
  float _x, _y;
public:
  MutableVector2(Vector2 vector2)
    : _x(vector2.x()), _y(vector2.y()) {}

  float x() const {
    return _x;
  }
  float y() const {
    return _y;
  }
  String toString() const {
    return "(Mutable," + String(_x) + ", " + String(_y) + ")";
  }
  Vector2 toVector2() const {
    return Vector2(x(), y());
  }
  bool operator==(const Vector2 &other) {
    return (_x == other.x() && _y == other.y());
  }
  bool operator!=(const Vector2 &other) {
    return (_x != other.x() || _y != other.y());
  }
  bool operator==(const MutableVector2 &other) {
    return (_x == other._x && _y == other._y);
  }
  bool operator!=(const MutableVector2 &other) {
    return (_x != other._x || _y != other._y);
  }
};

class RobotState {  //SIMPLIFIED VERSION
public:
  RobotState(
    Vector2 ballPos,
    Vector2 myPos,
    Vector2 partnerPos)
    : _ballPos(ballPos),
      _myPos(myPos),
      _partnerPos(partnerPos) {}

  Vector2 ballPos() const {
    return _ballPos;
  }
  Vector2 myPos() const {
    return _myPos;
  }
  Vector2 partnerPos() const {
    return _partnerPos;
  }

  String toString() const {
    return "RobotState (ballPos: " + _ballPos.toString() + " myPos: " + _myPos.toString() + " partnerPos " + _partnerPos.toString() + ")";
  }

private:
  const Vector2 _ballPos, _myPos, _partnerPos;
};

///////// END GLOBAL CLASSES

class LidarPoint {
public:
  LidarPoint(uint16_t distance, uint8_t intensity)
    : _distance(distance), _intensity(intensity) {}

  LidarPoint &operator=(const LidarPoint &) = delete;
  //getters
  uint16_t distance() const {
    return _distance;
  }  //distance from the center of the lidar
  uint8_t intensity() const {
    return _intensity;
  }

  static uint16_t getStep(uint16_t startAngle, uint16_t endAngle, unsigned int lenMinusOne = 11) {
    return (endAngle - startAngle) / lenMinusOne;
  }

  uint16_t getAngle(uint16_t startAngle, uint16_t step, unsigned int indice) {
    return startAngle + (step * indice);
  }

private:
  const uint16_t _distance;
  const uint8_t _intensity;
};

/*
class CircularLidarPointsBuffer {
private:
  LidarPoint *buffer;
  int size;
  int index;

public:
  CircularLidarPointsBuffer(int bufferSize)
    : size(bufferSize), index(0) {
    buffer = MyClass[size];
  }

  ~CircularLidarPointsBuffer() {
    delete[] buffer;
  }

  void addValue(const LidarPoint &newValue) {
    buffer[index] = newValue;
    index = (index + 1) % size;
  }

  LidarPoint &operator[](int i) {
    // Permet d'accéder aux instances de la classe avec l'opérateur []
    int adjustedIndex = (index + i) % size;
    return buffer[adjustedIndex];
  }
};*/

static const uint8_t crcTable[256] = {
  0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3,
  0xae, 0xf2, 0xbf, 0x68, 0x25, 0x8b, 0xc6, 0x11, 0x5c, 0xa9, 0xe4, 0x33,
  0x7e, 0xd0, 0x9d, 0x4a, 0x07, 0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f, 0xb8,
  0xf5, 0x1f, 0x52, 0x85, 0xc8, 0x66, 0x2b, 0xfc, 0xb1, 0xed, 0xa0, 0x77,
  0x3a, 0x94, 0xd9, 0x0e, 0x43, 0xb6, 0xfb, 0x2c, 0x61, 0xcf, 0x82, 0x55,
  0x18, 0x44, 0x09, 0xde, 0x93, 0x3d, 0x70, 0xa7, 0xea, 0x3e, 0x73, 0xa4,
  0xe9, 0x47, 0x0a, 0xdd, 0x90, 0xcc, 0x81, 0x56, 0x1b, 0xb5, 0xf8, 0x2f,
  0x62, 0x97, 0xda, 0x0d, 0x40, 0xee, 0xa3, 0x74, 0x39, 0x65, 0x28, 0xff,
  0xb2, 0x1c, 0x51, 0x86, 0xcb, 0x21, 0x6c, 0xbb, 0xf6, 0x58, 0x15, 0xc2,
  0x8f, 0xd3, 0x9e, 0x49, 0x04, 0xaa, 0xe7, 0x30, 0x7d, 0x88, 0xc5, 0x12,
  0x5f, 0xf1, 0xbc, 0x6b, 0x26, 0x7a, 0x37, 0xe0, 0xad, 0x03, 0x4e, 0x99,
  0xd4, 0x7c, 0x31, 0xe6, 0xab, 0x05, 0x48, 0x9f, 0xd2, 0x8e, 0xc3, 0x14,
  0x59, 0xf7, 0xba, 0x6d, 0x20, 0xd5, 0x98, 0x4f, 0x02, 0xac, 0xe1, 0x36,
  0x7b, 0x27, 0x6a, 0xbd, 0xf0, 0x5e, 0x13, 0xc4, 0x89, 0x63, 0x2e, 0xf9,
  0xb4, 0x1a, 0x57, 0x80, 0xcd, 0x91, 0xdc, 0x0b, 0x46, 0xe8, 0xa5, 0x72,
  0x3f, 0xca, 0x87, 0x50, 0x1d, 0xb3, 0xfe, 0x29, 0x64, 0x38, 0x75, 0xa2,
  0xef, 0x41, 0x0c, 0xdb, 0x96, 0x42, 0x0f, 0xd8, 0x95, 0x3b, 0x76, 0xa1,
  0xec, 0xb0, 0xfd, 0x2a, 0x67, 0xc9, 0x84, 0x53, 0x1e, 0xeb, 0xa6, 0x71,
  0x3c, 0x92, 0xdf, 0x08, 0x45, 0x19, 0x54, 0x83, 0xce, 0x60, 0x2d, 0xfa,
  0xb7, 0x5d, 0x10, 0xc7, 0x8a, 0x24, 0x69, 0xbe, 0xf3, 0xaf, 0xe2, 0x35,
  0x78, 0xd6, 0x9b, 0x4c, 0x01, 0xf4, 0xb9, 0x6e, 0x23, 0x8d, 0xc0, 0x17,
  0x5a, 0x06, 0x4b, 0x9c, 0xd1, 0x7f, 0x32, 0xe5, 0xa8
};

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

float _distance(Vector2 point1, Vector2 point2) {
  return _distance(point1.x(), point1.y(), point2.x(), point2.y());
}
float _distance(float point1X, float point1Y, float point2X, float point2Y) {
  return sqrt(sq(point1X - point1Y) + sq(point2X - point2Y));
}

RobotState _robotState = RobotState(
  Vector2(1, 1),  // ballPos
  Vector2(2, 2),  // myPos
  Vector2(3, 3)   // partnerPos
);

void setup() {
  SerialDebug.begin(230400);
  SerialLidar.begin(230400);
  SerialDebug.println("\ntest");
}

//Temporary global variables until lidar and motor codes are linked (due to Vector2's immutability)
const int sign = 1;
uint16_t sumX = 0;
uint16_t sumY = 0;
uint8_t numberSum = 0;
float cornersX[4] = {};
float cornersY[4] = {};
float maxDistance[4] = {};

void loop() {
  if (!SerialLidar.find("T,")) {  // equivalent en char de 84 44 (decimal)
    SerialDebug.println("error, no header-verlen found in RX for the lidar LD19");
  } else {
    // The previous instruction (find) jumped to the beginning of the information
    // Now the stream is aligned
    byte buffer[45];
    size_t nbrBytesReceived = SerialLidar.readBytes(buffer, 45);
    if (nbrBytesReceived != 45) {
      SerialDebug.println("error, wrong number of bytes received (" + String(nbrBytesReceived) + ")");
    } else {
      uint16_t speed = _get2BytesLsbMsb(buffer, 0);
      uint16_t startAngle = _get2BytesLsbMsb(buffer, 2);

      LidarPoint data[] = { //no for loop possible due to 'const' in LidarPoint class
                            LidarPoint(_get2BytesLsbMsb(buffer, 4), buffer[6]),
                            LidarPoint(_get2BytesLsbMsb(buffer, 7), buffer[9]),
                            LidarPoint(_get2BytesLsbMsb(buffer, 10), buffer[12]),
                            LidarPoint(_get2BytesLsbMsb(buffer, 13), buffer[15]),
                            LidarPoint(_get2BytesLsbMsb(buffer, 16), buffer[18]),
                            LidarPoint(_get2BytesLsbMsb(buffer, 19), buffer[21]),
                            LidarPoint(_get2BytesLsbMsb(buffer, 22), buffer[24]),
                            LidarPoint(_get2BytesLsbMsb(buffer, 25), buffer[27]),
                            LidarPoint(_get2BytesLsbMsb(buffer, 28), buffer[30]),
                            LidarPoint(_get2BytesLsbMsb(buffer, 31), buffer[33]),
                            LidarPoint(_get2BytesLsbMsb(buffer, 34), buffer[36]),
                            LidarPoint(_get2BytesLsbMsb(buffer, 37), buffer[39])
      };

      uint16_t endAngle = _get2BytesLsbMsb(buffer, 40);
      uint16_t timestamp = _get2BytesLsbMsb(buffer, 42);
      uint8_t crcCheck = buffer[44];

      uint8_t crcCal = _calCRC8FromBuffer(buffer, 44);

      if (crcCal == crcCheck) {
        uint16_t step = LidarPoint::getStep(startAngle, endAngle);
        for (unsigned int i = 0; i < 12; i++) {
          uint16_t angleRadians = ((data[i].getAngle(startAngle, step, i) / 100) / 180) * PI;
          // cos and sin are inverted because the direction of rotation is indirect and we start at pi/2
          Vector2 pointRefRobot = Vector2(
            data[i].distance() * sin(angleRadians),
            data[i].distance() * cos(angleRadians));

          //average
          sumX += pointRefRobot.x();
          sumY += pointRefRobot.y();
          numberSum++;

          //max TODO
          float distanceOrigin = _distance(Vector2(0, 0), pointRefRobot);
          for (unsigned int priority = 0; i < 4; i++) {  //find the 4 largest values
            if (distanceOrigin > maxDistance[priority]) {
              maxDistance[priority] = distanceOrigin;
              cornersX[priority] = pointRefRobot.x();
              cornersY[priority] = pointRefRobot.y();
              break;
            }
          }

          if (angleRadians / PI * 180 <= step) {  //a complete turn is performed, (why ?)
            //center
            Vector2 centerRefRobot = Vector2(
              sign * sumX / numberSum,
              sign * sumY / numberSum);

            //TODO
            for (int j = 0; j < 4; j++) {
              for (int k = 0; k < 4; k++) {
                float distanceCorner = _distance(cornersX[j], cornersY[j], cornersX[k], cornersY[k]);  //note : distance(A, B) = distance(B, A) !!

                if (distanceCorner >= 2230 && distanceCorner <= 2630) {  // Voir dimensions terrain
                  float orientation = sign * atan(cornersY[j] - cornersY[k] / cornersX[j] - cornersX[k]);
                }
              }
            }

            //reset
            sumX = 0;
            sumY = 0;
            numberSum = 0;
            cornersX[4] = {};
            cornersY[4] = {};
            maxDistance[4] = {};
          }
        }
      }
    }
  }
}
