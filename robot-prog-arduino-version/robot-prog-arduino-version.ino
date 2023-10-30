//AVOID MULTI-FILES .ino : They are just concatenated in ALPHABETIC ORDER -> bad idea

//TODO: changer le code en multi-fichiers en utilisant des namespace pour contourner les barrières du .ino ou séparer directement en .hpp et .cpp si cela fonctionne

class Vector2 {
  public:
  Vector2(float x, float y) : _x(x), _y(y) {}

  float x() const {
    return _x;
  }
  float y() const {
    return _y;
  }
  String toString() const {
    return "(" + String(_x) + ", " + String(_y) + ")";
  }
  private:
  const float _x, _y;
};

class GlobalParameters {
  public:
    GlobalParameters(
        float fieldLength,
        float fieldDepth,
        float spaceBeforeLineSide,
        float goalWidth
      ):
      _fieldLength(fieldLength),
      _fieldDepth(fieldDepth),
      _spaceBeforeLineSide(spaceBeforeLineSide),
      _goalWidth(goalWidth)
    {}

    float fieldLength() const {
      return _fieldLength;
    }
    float fieldDepth() const {
      return _fieldDepth;
    }
    float spaceBeforeLineSide() const {
      return _spaceBeforeLineSide;
    }
    float goalWidth() const {
      return _goalWidth;
    }

  private:
    const float _fieldLength;
    const float _fieldDepth;
    const float _spaceBeforeLineSide;
    const float _goalWidth;
};

class RobotState {
  public:
  RobotState(Vector2 ballPos, Vector2 myPos, Vector2 partnerPos): _ballPos(ballPos), _myPos(myPos), _partnerPos(partnerPos) {}

  Vector2 ballPos() const {
    return _ballPos;
  }
  Vector2 myPos() const {
    return _myPos;
  }
  Vector2 partnerPos() const {
    return _partnerPos;
  }

  private:
    const Vector2 _ballPos, _myPos, _partnerPos;
};

//Required because arduino doesn't support arrays
class Range {
  public:
  Range(float from, float to) : _from(from), _to(to) {}

  String toString() const {
    return "range from [" + String(_from) + " to " + String(_to) + "]";
  }

  private:
  const float _from, _to;
};

Range reboundGetRange(GlobalParameters globalParameters, RobotState robotState) {
  return reboundGetRange(
        globalParameters.fieldLength() /2,
        globalParameters.goalWidth() /2,
        globalParameters.fieldDepth() - robotState.myPos().y(),
        (globalParameters.fieldLength() /2) - robotState.myPos().x()
    );
};

Range reboundGetRange(float halfWidth, float halfWidthGoal, float depthFromRobot, float robotCenterDistance) {
  float val = ((2*halfWidth) - robotCenterDistance - halfWidthGoal) / depthFromRobot;
  Serial.println("before atan : " + String(val, 10));
  Serial.println("after atan : " + String(atan(val), 10));
    return Range(
      atan(((2*halfWidth) - robotCenterDistance - halfWidthGoal) / depthFromRobot)*(PI/180),
      0
    );
};

void setup() {
  // put your setup code here, to run once:
  GlobalParameters globalParameters = GlobalParameters(5, 10, 0, 1);
  RobotState robotState = RobotState(
    Vector2(1, 1),
    Vector2(2, 2),
    Vector2(3, 3)
    );
  Serial.begin(9600);
  Serial.println(reboundGetRange(globalParameters, robotState).toString());
}

void loop() {
  // put your main code here, to run repeatedly:

}
