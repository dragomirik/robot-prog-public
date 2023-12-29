//AVOID MULTI-FILES .ino : They are just concatenated in ALPHABETIC ORDER -> bad idea

//TODO: changer le code en multi-fichiers en utilisant des namespace pour contourner les barrières du .ino ou séparer directement en .hpp et .cpp si cela fonctionne

#define SerialDebug Serial
#define SerialCam Serial

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

  bool operator== (const Vector2 &other) {
    return (_x == other._x && _y == other._y);
  }
  bool operator!= (const Vector2 &other) {
    return (_x != other._x || _y != other._y);
  }

  private:
  const float _x, _y;
};

class Vector2OrError {
  public:
  Vector2OrError(String message):
    _errorMessage(message),
    _instanceVector2(Vector2(-1, -1)),
    _hasError(true)
    {
      SerialDebug.println("Warning, unraised error :" + message);
    }
  
  Vector2OrError(Vector2 instance):
    _errorMessage(""),
    _instanceVector2(instance),
    _hasError(false)
    {};
  
  bool isError() const {
    return _hasError;
  }

  Vector2 getVector2() const {
    return _instanceVector2;
  }

  String errorMessage() const {
    return _errorMessage;
  }

  Vector2 defaultIfError(Vector2 defaultVal) const {
    if (isError()) {
      return defaultVal;
    } else {
      return getVector2();
    }
  }

  private:
  const String _errorMessage;
  const Vector2 _instanceVector2;
  const bool _hasError;
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
  
  static RobotState fromString(RobotState defaultValues, String values) {
    Vector2OrError b = RobotState::splitLastUpdate(values, 'b');
    Vector2OrError m = RobotState::splitLastUpdate(values, 'm');
    Vector2OrError p = RobotState::splitLastUpdate(values, 'p');
    return RobotState(
      b.defaultIfError(defaultValues.ballPos()),
      m.defaultIfError(defaultValues.myPos()),
      p.defaultIfError(defaultValues.partnerPos())
    );
  }

  static Vector2OrError splitLastUpdate(String values, char charId) {
    size_t pos = values.lastIndexOf(charId);
    if (pos == -1) {
      return Vector2OrError("error RobotState splitLastUpdate: no '" + String(charId) + "' found in '" + values + "'");
    } else {
      Vector2OrError val = RobotState::splitFirstVector(values.substring(pos + 1));
      if (val.isError()) {
        return Vector2OrError("error RobotState splitLastUpdate, error in splitFirstVector: " + val.errorMessage());
      } else {
        return val;
      }
    }
  }
  static Vector2OrError splitFirstVector(String part) {
    String x_num;
    String y_num;
    bool add_x = true;
    for (int i = 0; i < part.length(); i++) {
      char character = part.charAt(i);
      if (isDigit(character) || character == '.') {
        if (add_x) {
          x_num += character;
        } else {
          y_num += character;
        }
      } else if (character == ',') {
        if (add_x) {
          add_x = false;
        } else {
          return Vector2OrError("error RobotState splitFirstVector, several characters ','");
        }
      } else if (character == 'b' || character == 'm' || character == 'p') {
        break;
      } else {
        return Vector2OrError("error RobotState splitFirstVector, unknown character '" + String(character) + "'");
      }
    }
    return Vector2OrError(Vector2(
      x_num.toFloat(),
      y_num.toFloat()
    ));
  }

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

    MotorMov frontRight() const {
      return _frontRight;
    }

    MotorMov frontLeft() const {
      return _frontLeft;
    }

    MotorMov backRight() const {
      return _backRight;
    }

    MotorMov backLeft() const {
      return _backLeft;
    }

  private:
    const MotorMov _frontRight;
    const MotorMov _frontLeft;
    const MotorMov _backRight;
    const MotorMov _backLeft;
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
  float valAtan = atan(val);
  // SerialDebug.println("halfWidth L " + String(halfWidth, 10));
  // SerialDebug.println("robotCenterDistance b " + String(robotCenterDistance, 10));
  // SerialDebug.println("halfWidthGoal l " + String(halfWidthGoal, 10));
  // SerialDebug.println("depthFromRobot d " + String(depthFromRobot, 10));
  // SerialDebug.println("before atan " + String(val, 10));
  // SerialDebug.println("after atan " + String(valAtan, 10));
  // SerialDebug.println("force degree " + String(valAtan*180/PI, 10));
  // SerialDebug.println("force radians " + String(valAtan*PI/180, 10));
  return Range(
    atan(((2*halfWidth) - robotCenterDistance - halfWidthGoal) / depthFromRobot)*180/PI,
    0
  );
};

void setup() {
  SerialDebug.begin(115200);
  SerialCam.begin(115200);
  SerialDebug.println("test");
}

void loop() {
  SerialDebug.println("t");
  SerialDebug.println(RobotState::fromString(
    RobotState(Vector2(0, 0), Vector2(0, 0), Vector2(0, 0)),
    "b622,355.456m8.6gf4556,4445p11.45,2.6.4"
    ).toString());
}

/*/

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
    
  SerialDebug.begin(115200);
  SerialDebug.println("test");
}

String receivedMessage;
unsigned int nbr = 0;

void loop() {
  // put your main code here, to run repeatedly:
  while (SerialCam.available() > 0) {
    char receivedChar = SerialCam.read();
    if (receivedChar == '\n') {
      if (receivedMessage != "Hello World") {
        SerialDebug.println(receivedMessage);
      } else {
        nbr += 1;
        if (nbr % 1000 == 0) {
          SerialDebug.println(nbr);
        }
      }
      receivedMessage = "";
    } else {
      receivedMessage += receivedChar;
    }
  }
}

*/