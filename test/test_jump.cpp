#include <iostream>
#include <set>
#include "gtest/gtest.h"

#include "jps/jps.hpp"

using namespace std;
using namespace JPS;

bool is_valid1(const Point & p) {
  // left figure
  if (p.x() < 0 || p.x() > 6 || p.y() < 0 || p.y() > 4 || p.z() < 0 || p.z() > 0)
    return false;

  if (p.x() == 5 && p.y() == 2)
    return false;

  if (p.x() == 0 && p.y() == 2)
    return false;

  if (p.x() == 0 && p.y() == 0)
    return false;

  return true;
}

bool is_valid2(const Point & p) {
  // right figure
  if (p.x() < 0 || p.x() > 6 || p.y() < 0 || p.y() > 4 || p.z() < 0 || p.z() > 0)
    return false;

  if (p.x() == 1 && p.y() <= 1)
    return false;

  if (p.x() == 5 && p.y() >= 1 && p.y() <= 3)
    return false;

  return true;
}


TEST(TestSuite, testJump)  {
  // Examples from 
  //
  // Harabor, D., & Grastien, A. (2011). 
  // Online graph pruning for pathfinding on grid maps. 
  // Proceedings of the National Conference on Artificial Intelligence, 2, 1114–1119.
  //
  // we block z=-1 and z=1 to force a 2D problem

  auto[succ1, point1] = jump({0,1,0}, {1,0,0}, {6,3,0}, is_valid1);

  EXPECT_EQ(succ1, true);
  EXPECT_EQ(point1, Point({5,1,0}));

  auto[succ2, point2] = jump({2,3,0}, {1,0,0}, {6,3,0}, is_valid1);

  EXPECT_EQ(succ2, true);
  EXPECT_EQ(point2, Point({5,3,0}));

  auto[succ3, point3] = jump({2,1,0}, {1,1,0}, {6,3,0}, is_valid1);

  EXPECT_EQ(succ3, true);
  EXPECT_EQ(point3, Point({4,3,0}));

  auto[succ4, point4] = jump({0,1,0}, {1,1,0}, {6,3,0}, is_valid2);

  EXPECT_EQ(succ4, true);
  EXPECT_EQ(point4, Point({1,2,0}));
}
