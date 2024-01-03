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
      }
    }
  }
}

uint16_t _get2BytesLsbMsb(byte buffer[], int index) {
    return (buffer[index + 1] << 8) | buffer[index];
}

float getDistance(Vector2 point1, Vector2 point2) {
  return sqrt( sq(point1.x() - point2.x()) + sq(point1.y() - point2.y()) );
}