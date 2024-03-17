#include "lidar.h"
#include "movements.h"
#include "states.h"
#include "strategy.h"
#include "utilities.h"

const FieldProperties fieldProperties = FieldProperties(
  5,   // fieldLength
  10,  // fieldDepth
  0,   // spaceBeforeLineSide
  2    // goalWidth
);

void setup() {
  SerialDebug.begin(230400);
  SerialCam.begin(115200);
  SerialLidar.begin(230400);
}

void loop() {
  SerialDebug.println("OK!");
}