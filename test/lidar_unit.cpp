#include <gtest/gtest.h>

#include "../src/lidar.h"

TEST(LidarGetter, CircularLidarPointsBufferAddValueMinimum) {
  CircularLidarPointsBuffer clpb {5};
  clpb.addValue(LidarPoint(1, 1, 1));
  clpb.addValue(LidarPoint(2, 2, 2));
  ASSERT_EQ(clpb.getValue(0).angle(), 1);
  ASSERT_EQ(clpb.getValue(1).angle(), 2);
  ASSERT_EQ(clpb.sizeFilled(), 2);
  clpb.addValue(LidarPoint(3, 3, 3));
  clpb.addValue(LidarPoint(4, 4, 4));
  ASSERT_EQ(clpb.sizeFilled(), 4);
  ASSERT_TRUE(clpb.existValue(0));
  ASSERT_TRUE(clpb.existValue(3));
  ASSERT_FALSE(clpb.existValue(4));
  ASSERT_FALSE(clpb.existValue(5));
  ASSERT_EQ(clpb.getValue(0).angle(), 1);
  ASSERT_EQ(clpb.getValue(1).angle(), 2);
  ASSERT_EQ(clpb.getValue(2).angle(), 3);
  ASSERT_EQ(clpb.getValue(3).angle(), 4);
}

TEST(LidarGetter, CircularLidarPointsBufferAddValueMaximum) {
  CircularLidarPointsBuffer clpb {5};
  clpb.addValue(LidarPoint(1, 1, 1));
  clpb.addValue(LidarPoint(2, 2, 2));
  ASSERT_EQ(clpb.getValue(0).angle(), 1);
  ASSERT_EQ(clpb.getValue(1).angle(), 2);
  ASSERT_EQ(clpb.sizeFilled(), 2);
  clpb.addValue(LidarPoint(3, 3, 3));
  clpb.addValue(LidarPoint(4, 4, 4));
  clpb.addValue(LidarPoint(5, 5, 5));
  clpb.addValue(LidarPoint(6, 6, 6));
  clpb.addValue(LidarPoint(7, 7, 7));
  ASSERT_EQ(clpb.sizeFilled(), 5);
  ASSERT_EQ(clpb.getValue(0).angle(), 6);
  ASSERT_EQ(clpb.index(), 2);
  ASSERT_EQ(clpb.getValue(1).angle(), 7);
  ASSERT_EQ(clpb.getValue(2).angle(), 3);
  ASSERT_EQ(clpb.getValue(3).angle(), 4);
}