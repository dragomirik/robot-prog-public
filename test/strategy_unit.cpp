#include <gtest/gtest.h>

#include "../src/strategy.h"

TEST(FutureAction, accessToData) {
  FutureAction fA = FutureAction(Vector2(50, 13), true);
  ASSERT_EQ(fA.changeMove(), true);
  ASSERT_EQ(fA.target(), Vector2(50, 13));
  ASSERT_EQ(fA.activeKicker(), true);
}

TEST(FutureAction, accessToInfos) {
  FutureAction fA = FutureAction(true);
  ASSERT_EQ(fA.changeMove(), false);
  ASSERT_EQ(fA.target(), Vector2(0, 0));
  ASSERT_EQ(fA.activeKicker(), true);
}

TEST(FutureAction, leavingField) {
  FieldProperties fP = FieldProperties(
      243,               // fieldLength
      182,               // fieldWidth
      12,                // spaceBeforeLineSide
      60,                // goalWidth
      Vector2(0, -115),  // myGoalPos
      Vector2(0, 115),   // enemyGoalPos
      9,                 // robotRadius
      2                  // ballRadius
  );

  RobotState cS0 = RobotState(
      Vector2(0, 0),      // ballPos
      Vector2(0, 0),      // myPos
      Vector2(0, 0),      // partnerPos
      Vector2(500, 500),  // myGoalPos
      Vector2(500, 500)   // enemyGoalPos
  );

  ASSERT_FALSE(leavingField(fP, cS0));

  RobotState cS1 = RobotState(
      Vector2(0, 0),      // ballPos
      Vector2(-83, 0),    // myPos
      Vector2(0, 0),      // partnerPos
      Vector2(500, 500),  // myGoalPos
      Vector2(500, 500)   // enemyGoalPos
  );

  ASSERT_TRUE(leavingField(fP, cS1));

  RobotState cS2 = RobotState(
      Vector2(0, 0),      // ballPos
      Vector2(-81, 0),    // myPos
      Vector2(0, 0),      // partnerPos
      Vector2(500, 500),  // myGoalPos
      Vector2(500, 500)   // enemyGoalPos
  );

  ASSERT_FALSE(leavingField(fP, cS2));

  RobotState cS3 = RobotState(
      Vector2(0, 0),      // ballPos
      Vector2(83, 0),     // myPos
      Vector2(0, 0),      // partnerPos
      Vector2(500, 500),  // myGoalPos
      Vector2(500, 500)   // enemyGoalPos
  );

  ASSERT_TRUE(leavingField(fP, cS3));

  RobotState cS4 = RobotState(
      Vector2(0, 0),      // ballPos
      Vector2(81, 0),     // myPos
      Vector2(0, 0),      // partnerPos
      Vector2(500, 500),  // myGoalPos
      Vector2(500, 500)   // enemyGoalPos
  );

  ASSERT_FALSE(leavingField(fP, cS4));

  RobotState cS5 = RobotState(
      Vector2(0, 0),      // ballPos
      Vector2(81, 113),   // myPos
      Vector2(0, 0),      // partnerPos
      Vector2(500, 500),  // myGoalPos
      Vector2(500, 500)   // enemyGoalPos
  );

  ASSERT_TRUE(leavingField(fP, cS5));

  RobotState cS6 = RobotState(
      Vector2(0, 0),      // ballPos
      Vector2(81, 111),   // myPos
      Vector2(0, 0),      // partnerPos
      Vector2(500, 500),  // myGoalPos
      Vector2(500, 500)   // enemyGoalPos
  );

  ASSERT_FALSE(leavingField(fP, cS6));

  RobotState cS7 = RobotState(
      Vector2(0, 0),      // ballPos
      Vector2(81, -113),  // myPos
      Vector2(0, 0),      // partnerPos
      Vector2(500, 500),  // myGoalPos
      Vector2(500, 500)   // enemyGoalPos
  );

  ASSERT_TRUE(leavingField(fP, cS7));

  RobotState cS8 = RobotState(
      Vector2(0, 0),      // ballPos
      Vector2(81, -111),  // myPos
      Vector2(0, 0),      // partnerPos
      Vector2(500, 500),  // myGoalPos
      Vector2(500, 500)   // enemyGoalPos
  );

  ASSERT_FALSE(leavingField(fP, cS8));

  RobotState cS9 = RobotState(
      Vector2(0, 0),     // ballPos
      Vector2(0, 0),     // myPos
      Vector2(0, 0),     // partnerPos
      Vector2(0, 0),     // myGoalPos
      Vector2(500, 500)  // enemyGoalPos
  );

  ASSERT_TRUE(leavingField(fP, cS9));

  RobotState cS10 = RobotState(
      Vector2(0, 0),      // ballPos
      Vector2(0, 0),      // myPos
      Vector2(0, 0),      // partnerPos
      Vector2(500, 500),  // myGoalPos
      Vector2(0, 0)       // enemyGoalPos
  );

  ASSERT_TRUE(leavingField(fP, cS10));
}

TEST(FutureAction, targetInFrontOfRobot) {
  FieldProperties fP = FieldProperties(
      243,               // fieldLength
      182,               // fieldWidth
      12,                // spaceBeforeLineSide
      60,                // goalWidth
      Vector2(0, 0),  // myGoalPos
      Vector2(0, 0),   // enemyGoalPos
      9,                 // robotRadius
      2                  // ballRadius
  );

  RobotState cS = RobotState(
      Vector2(0, 0),      // ballPos
      Vector2(0, 0),      // myPos
      Vector2(0, 0),      // partnerPos
      Vector2(0, 0),  // myGoalPos
      Vector2(0, 0)   // enemyGoalPos
  );

  ASSERT_TRUE(targetInFrontOfRobot(fP, cS, Vector2(0, 50)));
  ASSERT_FALSE(targetInFrontOfRobot(fP, cS, Vector2(0, 0)));
  ASSERT_FALSE(targetInFrontOfRobot(fP, cS, Vector2(0, -50)));
  ASSERT_TRUE(targetInFrontOfRobot(fP, cS, Vector2(0, 20)));
  ASSERT_FALSE(targetInFrontOfRobot(fP, cS, Vector2(0, 19)));
}

TEST(FutureAction, targetCenterOfRobot) {
  FieldProperties fP = FieldProperties(
      243,               // fieldLength
      182,               // fieldWidth
      12,                // spaceBeforeLineSide
      60,                // goalWidth
      Vector2(0, 0),  // myGoalPos
      Vector2(0, 0),   // enemyGoalPos
      9,                 // robotRadius
      2                  // ballRadius
  );

  RobotState cS = RobotState(
      Vector2(0, 0),      // ballPos
      Vector2(0, 0),      // myPos
      Vector2(0, 0),      // partnerPos
      Vector2(0, 0),  // myGoalPos
      Vector2(0, 0)   // enemyGoalPos
  );

  ASSERT_TRUE(targetCenterOfRobot(fP, cS, Vector2(0,0)));
  ASSERT_TRUE(targetCenterOfRobot(fP, cS, Vector2(-10,0)));
  ASSERT_TRUE(targetCenterOfRobot(fP, cS, Vector2(10,0)));
  ASSERT_FALSE(targetCenterOfRobot(fP, cS, Vector2(-11,0)));
  ASSERT_FALSE(targetCenterOfRobot(fP, cS, Vector2(11,0)));
}