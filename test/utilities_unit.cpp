#include <gtest/gtest.h>

#include "../src/utilities.h"

TEST(Vector2, f1Norm) {
  ASSERT_EQ(Vector2(1, 0), Vector2(0, 0));
}