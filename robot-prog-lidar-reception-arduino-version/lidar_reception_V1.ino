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

  LidarPoint() : _distance(_findNext2BytesLSBMSBValue()), _intensity(SerialLidar.read()) {}

  private:
  const uint16_t _distance;
  const uint8_t _intensity;
};

void loop() {
  if (SerialLidar.available() >= 47) {
    if (!SerialLidar.find("T,")) { // equivalent en char de 84 44 (decimal)
      SerialDebug.println("error, no header-verlen found in RX for the lidar LD19");
    } else {
      // The previous instruction (find) jumped to the beginning of the information
      // Now the stream is aligned
      uint16_t speed = _findNext2BytesLSBMSBValue();
      uint16_t startAngle = _findNext2BytesLSBMSBValue();

      LidarPoint data[12]; //implicitly call _findNext2BytesLSBMSBValue()) and SerialLidar.read() 12x

      uint16_t endAngle = _findNext2BytesLSBMSBValue();
      uint16_t timestamp = _findNext2BytesLSBMSBValue();
      uint8_t crcCheck = SerialLidar.read();

      SerialDebug.println(timestamp);
    }
  }
}

uint16_t _findNext2BytesLSBMSBValue() {
  byte firstByte = SerialLidar.read();
  byte secondByte = SerialLidar.read();
  if (firstByte == -1 || secondByte == -1) {
    return -1;
  } else {
    return (secondByte << 8) | firstByte;
  }
}