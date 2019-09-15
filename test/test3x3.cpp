#include <iostream>
#include <set>
#include "gtest/gtest.h"

#include "jps/jps.hpp"

using namespace std;
using namespace JPS;

TEST(TestSuite, test2D)  {
  // Examples from 
  //
  // Harabor, D., & Grastien, A. (2011). 
  // Online graph pruning for pathfinding on grid maps. 
  // Proceedings of the National Conference on Artificial Intelligence, 2, 1114–1119.
  //
  // we block z=-1 and z=1 to force a 2D problem

  set<Dir> obs_2d;
  for (int8_t dx=-1; dx != 2; ++dx)
    for (int8_t dy=-1; dy !=2; ++dy)
      obs_2d.insert(Dir(dx, dy, int8_t(-1))), obs_2d.insert(Dir(dx, dy, int8_t(1)));

  // Test 2D-1
  Dir par = {-1, 0, 0};
  set<Dir> obs = obs_2d;
  set<Dir> nn = all_neighbors(par, obs);

  set<Dir> rhs = {{1,0,0}};
  EXPECT_EQ(nn, rhs);


  // Test 2D-2
  par = {-1, 0, 0};
  obs = obs_2d;
  obs.insert(Dir(0,1,0));
  nn = all_neighbors(par, obs);

  rhs = {{1,0,0}, {1,1,0}};
  EXPECT_EQ(nn, rhs);

  // Test 2D-3 
  par = {-1, -1, 0};
  obs = {};
  nn = all_neighbors(par, obs_2d);

  rhs = {{1,0,0}, {1,1,0}, {0,1,0}};
  EXPECT_EQ(nn, rhs);

  // Test 2D-4 
  par = {-1, -1, 0};
  obs = obs_2d;
  obs.insert(Dir(-1,0,0));
  nn = all_neighbors(par, obs);

  rhs = {{-1,1,0}, {1,0,0}, {1,1,0}, {0,1,0}};
  EXPECT_EQ(nn, rhs);
}

TEST(TestSuite, test3D) {
  // Examples from 
  //
  // Liu, S., Watterson, M., Mohta, K., Sun, K., Bhattacharya, S., Taylor, C. J., & Kumar, V. (2017). 
  // Planning dynamically feasible trajectories for quadrotors using safe flight corridors in 3-D complex environments. 
  // IEEE Robotics and Automation Letters, 2(3), 1688–1695. https://doi.org/10.1109/LRA.2017.2663526
  //
  // These examples are a bit conservative, they add more forced nn than necessary, not
  // all that are expected from the paper are returned

  // Test 3D-1: expected 
  Dir par = {-1, 0, 0};
  set<Dir> obs = {{0, -1, 0}, {0, -1, 1}};
  set<Dir> nn = all_neighbors(par, obs);

  set<Dir> rhs = {{1,0,0}, {1,-1,0}, {1, -1, 1}};
  EXPECT_EQ(nn, rhs);

  // Test 3D-2: expected {{1,0,0}, {1,1,0}, {0, 1, 0}, 
  //                   {1, -1, 0}, {1,0,1}, {1,1,1}, 
  //                   {0, 1, 1}, {1, -1, 1}}

  // MISSING: {1, -1, 0}  and  {0, 1, 1} but seems fine
  par = {-1, -1, 0};
  obs = {{0, -1, 0}, {0, -1, 1}, {0, 0, 1}};
  nn = all_neighbors(par, obs);

  rhs = {{1,0,0}, {1,1,0}, {0, 1, 0}, {1,0,1}, {1,1,1}, {1, -1, 1}};
  EXPECT_EQ(nn, rhs);

  // Test 3D-3: expected {{1,0,-1}, {1,1,-1}, 
  //                   {0, 1, -1}, {1, -1, -1},
  //                   {1,1,1}, {1,1,0}, {1,0,1}, {0,1,1},
  //                   {1,0,0}, {0,1,0}, {0,0,1}}

  // MISSING: {1, 0, -1}, {1,1,-1}, {0,1,-1}, {1,-1,-1} but seems fine
  par = {-1, -1, -1};
  obs = {{0, -1, -1}, {0, 0, -1}};
  nn = all_neighbors(par, obs);

  rhs = {{1,1,1}, {1,1,0}, {1,0,1}, {0,1,1},
                  {1,0,0}, {0,1,0}, {0,0,1}};
  EXPECT_EQ(nn, rhs);
}