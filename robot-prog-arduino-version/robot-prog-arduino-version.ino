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
  RobotState(
    Vector2 ballPos,
    Vector2 myPos,
    Vector2 partnerPos
    ):
    _ballPos(ballPos),
    _myPos(myPos),
    _partnerPos(partnerPos)
    {}

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

//TODO supprimer la mémorisation de sens en la remplaçant par une détection en temps réel grâce à FG
enum class Direction {forward, backward, stopped};

//TODO n'utilise pas FG (je ne sais pas comment faire), potentiellemnt remplacer _direction par une détection en directe grâce à FG
class MotorMov {
  public:
    MotorMov(
      uint8_t pinPWM,
      uint8_t pinCWCCW,
      uint8_t pinFG,
      uint8_t pinBRAKE
    ):_pinPWM(pinPWM),
      _pinCWCCW(pinCWCCW),
      _pinFG(pinFG),
      _pinBRAKE(pinBRAKE)
    {
      pinMode(_pinPWM, OUTPUT);
      pinMode(_pinCWCCW, OUTPUT);
      pinMode(_pinFG, INPUT);
      pinMode(_pinBRAKE, OUTPUT);
      _direction = Direction::stopped;
    }
  
  void stop() {
    _pwm(0);
    _brake(HIGH);
    _direction = Direction::stopped;
  }

  void move(int value) {
    if (value == 0) {
      stop();
    } else if (value > 0) {
      //forward
      if (_direction == Direction::backward) {
        stop();
      }
      _brake(LOW);
      _cwccw(LOW);
      _pwm(value);
      _direction = Direction::forward;
    } else {
      //backward
      if (_direction == Direction::forward) {
        stop();
      }
      _brake(LOW);
      _cwccw(HIGH);
      _pwm(-value);
      _direction = Direction::backward;
    }
  }

  private:
    const uint8_t _pinPWM;
    const uint8_t _pinCWCCW;
    const uint8_t _pinFG;
    const uint8_t _pinBRAKE;

    Direction _direction;//TODO remplacer par détection en direct via fg

    void _pwm(int value) const {
      analogWrite(_pinPWM, value);
    }

    void _cwccw(uint8_t value) const {
      digitalWrite(_pinCWCCW, value);
    }

    void _fg(uint8_t value) const {
      digitalWrite(_pinFG, value);
    }

    void _brake(uint8_t value) const {
      digitalWrite(_pinBRAKE, value);
    }
};

class Motors {
  public:
    Motors(
      MotorMov frontRight,
      MotorMov frontLeft,
      MotorMov backRight,
      MotorMov backLeft
    ):
      _frontRight(frontRight),
      _frontLeft(frontLeft),
      _backRight(backRight),
      _backLeft(backLeft)
    {}

    frontRight() const {
      return _frontRight;
    }

    frontLeft() const {
      return _frontLeft;
    }

    backRight() const {
      return _backRight;
    }

    backLeft() const {
      return _backLeft;
    }

  private:
    const MotorMov _frontRight;
    const MotorMov _frontLeft;
    const MotorMov _backRight;
    const MotorMov _backLeft;
}

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
  float valAtan = atan(val);
  // Serial.println("halfWidth L " + String(halfWidth, 10));
  // Serial.println("robotCenterDistance b " + String(robotCenterDistance, 10));
  // Serial.println("halfWidthGoal l " + String(halfWidthGoal, 10));
  // Serial.println("depthFromRobot d " + String(depthFromRobot, 10));
  // Serial.println("before atan " + String(val, 10));
  // Serial.println("after atan " + String(valAtan, 10));
  // Serial.println("force degree " + String(valAtan*180/PI, 10));
  // Serial.println("force radians " + String(valAtan*PI/180, 10));
  return Range(
    atan(((2*halfWidth) - robotCenterDistance - halfWidthGoal) / depthFromRobot)*180/PI,
    0
  );
};



void setup() {
  // put your setup code here, to run once:
  GlobalParameters globalParameters = GlobalParameters(
    5,    // fieldLength
    10,   // fieldDepth
    0,    // spaceBeforeLineSide
    2     // goalWidth
    );
  RobotState robotState = RobotState(
    Vector2(1, 1),  // ballPos
    Vector2(2, 2),  // myPos
    Vector2(3, 3)   // partnerPos
    );
  Serial.begin(9600);
  Serial.println("test");
  Serial.println(reboundGetRange(globalParameters, robotState).toString());
}

void loop() {
  // put your main code here, to run repeatedly:

}
