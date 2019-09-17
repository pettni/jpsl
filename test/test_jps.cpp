#include <iostream>
#include <set>
#include "gtest/gtest.h"

#include "jpsl/jpsl.hpp"

using namespace std;
using namespace JPSL;

// Example from https://zerowidth.com/2013/a-visual-explanation-of-jump-point-search.html
bool state_valid(const Point & p) {
  if (p.x() < 0 || p.x() >= 8 || p.y() < 0 || p.y() >= 5 || p.z() < 0 || p.z() > 0)
    return false;

  if (p.x() == 5 && p.y() < 3)
    return false;

  return true;
}

TEST(test_jps, solve_2d) {
  auto[path, cost] = plan({0,0,0}, {7,0,0}, &state_valid);
  vector<Point> ans = {{0,0,0}, {3,3,0}, {5,3,0}, {7,1,0}, {7,0,0}};
  EXPECT_EQ(path, ans);
  EXPECT_LE(cost, 10.075);
}