#define SerialDebug Serial
#define SerialLidar Serial2

void setup() {
  SerialDebug.begin(230400);
  SerialLidar.begin(230400);
  SerialDebug.println("\ntest");
}

class LidarPoint {
  public:
  LidarPoint(uint16_t distance, uint8_t intensity) : _distance(distance), _intensity(intensity) {}

  //getters
  uint16_t distance() const {return _distance;} //distance from the center of the lidar
  uint8_t intensity() const {return _intensity;}

  static uint16_t getStep(uint16_t startAngle, uint16_t endAngle, unsigned int lenMinusOne= 11) {
    return (endAngle - startAngle) / lenMinusOne;
  }

  uint16_t getAngle(uint16_t startAngle, uint16_t step, unsigned int indice) {
    return startAngle + (step * indice);
  }

  private:
  const uint16_t _distance;
  const uint8_t _intensity;
};

float sumX = 0;
float sumY = 0;
unsigned int compteur = 0;
float maxDistance1 = 0;
float maxDistance2 = 0;
float maxDistance3 = 0;
float maxDistance4 = 0;
Vector2 robotPos(0, 0);

void loop() {
  if (!SerialLidar.find("T,")) { // equivalent en char de 84 44 (decimal)
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

      LidarPoint data[] = {//no for loop possible due to 'const' in LidarPoint class
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
      
      SerialDebug.println(timestamp);
	  
	  
	  uint16_t step = LidarPoint::getStep(startAngle, endAngle);

      for(int i = 0; i < 12; i++) {
        // cos et sin sont inversés car le sens de rotation est indirect et on commence en pi/2
        float pointX = data[i].distance() * sin(data[i].getAngle(startAngle, step, i));
        float pointY = data[i].distance() * cos(data[i].getAngle(startAngle, step, i));
        Vector2 point(pointX, pointY);

        sumX += pointX;
        sumY += pointY;
        compteur++;

        float distanceOrigin = getDistance(point, robotPos);
        Vector2 corners[] = {};
		
		if (distanceOrigin > maxDistance1) {
            maxDistance1 = distanceOrigin;
            corners[0] = point;
        } else if (distanceOrigin > maxDistance2) {
            maxDistance2 = distanceOrigin;
            corners[1] = point;
        } else if (distanceOrigin > maxDistance3) {
            maxDistance3 = distanceOrigin;
            corners[2] = point;
        } else if (distanceOrigin > maxDistance4) {
            maxDistance4 = distanceOrigin;
            corners[3] = point;
        }
        
        if(data[i].getAngle(startAngle, step, i) <= step) {
          int sign = 1; // Le signe de la coordonnée en y du goal adverse, soit 1 ou -1
          float robotPosX = sign * sumX / compteur;
          float robotPosY = sign * sumY / compteur;
          Vector2 robotPos(robotPosX, robotPosY);

          sumX = 0;
          sumY = 0;
          compteur = 0;

          for (int j = 0; j < 4; j++) {
            for (int k = 0; k < 4; k++) {
              float distanceCorner = getDistance(corners[j], corners[k]);

              if(distanceCorner >= 223 && distanceCorner <= 263) { // Voir dimensions terrain
                float orientation = sign * atan(corners[j].y() - corners[k].y() / corners[j].x() - corners[k].x()); 
              }
            }
          }
        }

      byte crcArray[46];
      crcArray[0] = 0x54;
      crcArray[1] = 0x2C;
      for (int i = 2; i < 46; i++) {
        crcArray[i] = buffer[i - 2];
      }
      SerialDebug.println("########################################################################################################");
      SerialDebug.print("        ");
      printArray(buffer, 45);
      printArray(crcArray, 46);


      uint8_t crcCal = _calCRC8(crcArray, 46);
      
      if (crcCal != crcCheck) {
        SerialDebug.print(crcCal);
        SerialDebug.print(" != ");
        SerialDebug.print(crcCheck);
        SerialDebug.print("\n");
      } else {
        SerialDebug.print("\n");
      }
    }
  }
}

void printArray(byte tableau[], int taille) {
  for (int i = 0; i < taille; i++) {
    if (tableau[i] < 10) {
      Serial.print("  ");
    } else if (tableau[i] < 100) {
      Serial.print(" ");
    }
    SerialDebug.print(tableau[i]);
    SerialDebug.print(" ");
  }
  SerialDebug.println();
}

static const uint8_t crcTable[256]
={
0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3,
0xae, 0xf2, 0xbf, 0x68, 0x25, 0x8b, 0xc6, 0x11, 0x5c, 0xa9, 0xe4, 0x33,
0x7e, 0xd0, 0x9d, 0x4a, 0x07, 0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f, 0xb8,
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

uint8_t _calCRC8(uint8_t *p, uint8_t len){
  uint8_t crc = 0;
  uint16_t i;

      for (i = 0; i < len; i++){
      crc = crcTable[(crc ^ *p++) & 0xff];
      }

      return crc;
}

uint16_t _get2BytesLsbMsb(byte buffer[], int index) {
    return (buffer[index + 1] << 8) | buffer[index];
}

float getDistance(Vector2 point1, Vector2 point2) {
  return sqrt( sq(point1.x() - point2.x()) + sq(point1.y() - point2.y()) );
}