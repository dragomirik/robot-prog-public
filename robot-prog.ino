#include "utilities.h"

void setup() {
  Serial.begin(9600);
}

void loop() {
  Vector2 test = Vector2(1, 1);
  Serial.println(test.toString());
}