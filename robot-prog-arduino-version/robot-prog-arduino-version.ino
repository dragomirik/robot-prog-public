//AVOID MULTI-FILES .ino : They are just concatenated in ALPHABETIC ORDER -> bad idea

//TODO: changer le code en multi-fichiers en utilisant des namespace pour contourner les barrières du .ino ou séparer directement en .hpp et .cpp si cela fonctionne

class GlobalParameters {
  public:
  static const float fieldLength {70};
  static const float fieldDepth {100};
  static const float spaceBeforeLineSide {5};
  static const float goalWidth {10};
};

//Required because arduino doesn't support arrays
class Range {
  public:
  Range(float from, float to) {
    _from = from;
    _to = to;
  };

  String toString() {
    return "range from [" + String(_from) + " to " + String(_to) + "]";
  };

  private:
  float _from;
  float _to;
};

Range areboundGetRange() {
  return breboundGetRange(
        GlobalParameters::fieldLength /2,
        GlobalParameters::goalWidth /2,
        1,
        1
    );
};

Range breboundGetRange(float halfWidth, float halfWidthGoal, float depthFromRobot, float robotCenterDistance) {
    return Range(1, 0);
};

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.print(areboundGetRange().toString());
}

void loop() {
  // put your main code here, to run repeatedly:

}
