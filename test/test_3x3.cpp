#include <iostream>
#include <unordered_set>
#include "gtest/gtest.h"

#include "jpsl/jpsl.hpp"

using namespace std;
using namespace JPSL;

TEST(test_3x3, test2D)  {
  // Examples from 
  //
  // Harabor, D., & Grastien, A. (2011). 
  // Online graph pruning for pathfinding on grid maps. 
  // Proceedings of the National Conference on Artificial Intelligence, 2, 1114–1119.
  //
  // we block z=-1 and z=1 to force a 2D problem

  vector<Dir> obs_2d;
  for (int8_t dx=-1; dx != 2; ++dx)
    for (int8_t dy=-1; dy !=2; ++dy)
      obs_2d.push_back(Dir(dx, dy, int8_t(-1))), obs_2d.push_back(Dir(dx, dy, int8_t(1)));

  Point center(0,0,0);

  // Test 2D-1
  Dir par = {-1, 0, 0};
  vector<Dir> obs = obs_2d;
  vector<Dir> nn = all_neighbors(center, Point(-1,0,0), [center, obs] (const Point & p) {
    return find(obs.begin(), obs.end(), center.direction_to(p)) == obs.end();
  });

  vector<Dir> rhs = {{1,0,0}};
  sort(nn.begin(), nn.end());  
  sort(rhs.begin(), rhs.end());
  EXPECT_EQ(nn, rhs);

  // Compare forced
  vector<Dir> fn1 = forced_neighbors_slow(center, Point(-1,0,0), [center, obs] (const Point & p) {
    return find(obs.begin(), obs.end(), center.direction_to(p)) == obs.end();
  });    
  vector<Dir> fn2 = forced_neighbors_fast(center, Point(-1,0,0), [center, obs] (const Point & p) {
    return find(obs.begin(), obs.end(), center.direction_to(p)) == obs.end();
  });
  sort(fn1.begin(), fn1.end());  
  sort(fn2.begin(), fn2.end());    
  EXPECT_EQ(fn1, fn2);

  // Test 2D-2
  par = {-1, 0, 0};
  obs = obs_2d;
  obs.push_back(Dir(0,1,0));
  nn = all_neighbors(center, Point(-1,0,0), [center, obs] (const Point & p) {
    return find(obs.begin(), obs.end(), center.direction_to(p)) == obs.end();
  });

  rhs = {{1,0,0}, {1,1,0}};
  sort(nn.begin(), nn.end());  
  sort(rhs.begin(), rhs.end());
  EXPECT_EQ(nn, rhs);
  fn1 = forced_neighbors_slow(center, Point(-1,0,0), [center, obs] (const Point & p) {
    return find(obs.begin(), obs.end(), center.direction_to(p)) == obs.end();
  });    
  fn2 = forced_neighbors_fast(center, Point(-1,0,0), [center, obs] (const Point & p) {
    return find(obs.begin(), obs.end(), center.direction_to(p)) == obs.end();
  });
  sort(fn1.begin(), fn1.end());  
  sort(fn2.begin(), fn2.end());    
  EXPECT_EQ(fn1, fn2);

  // Test 2D-3 
  par = {-1, -1, 0};
  obs = obs_2d;
  nn = all_neighbors(center, Point(-1,-1,0), [center, obs] (const Point & p) {
    return find(obs.begin(), obs.end(), center.direction_to(p)) == obs.end();
  });
  rhs = {{1,0,0}, {1,1,0}, {0,1,0}};
  sort(nn.begin(), nn.end());  
  sort(rhs.begin(), rhs.end());
  EXPECT_EQ(nn, rhs);

  fn1 = forced_neighbors_slow(center, Point(-1,-1,0), [center, obs] (const Point & p) {
    return find(obs.begin(), obs.end(), center.direction_to(p)) == obs.end();
  });    
  fn2 = forced_neighbors_fast(center, Point(-1,-1,0), [center, obs] (const Point & p) {
    return find(obs.begin(), obs.end(), center.direction_to(p)) == obs.end();
  });
  sort(fn1.begin(), fn1.end());  
  sort(fn2.begin(), fn2.end());    
  EXPECT_EQ(fn1, fn2);

  // Test 2D-4 
  par = {-1, -1, 0};
  obs = obs_2d;
  obs.push_back(Dir(-1,0,0));
  nn = all_neighbors(center, Point(-1,-1,0), [center, obs] (const Point & p) {
    return find(obs.begin(), obs.end(), center.direction_to(p)) == obs.end();
  });

  rhs = {{-1,1,0}, {1,0,0}, {1,1,0}, {0,1,0}};
  sort(nn.begin(), nn.end());  
  sort(rhs.begin(), rhs.end());
  EXPECT_EQ(nn, rhs);

  fn1 = forced_neighbors_slow(center, Point(-1,-1,0), [center, obs] (const Point & p) {
    return find(obs.begin(), obs.end(), center.direction_to(p)) == obs.end();
  });    
  fn2 = forced_neighbors_fast(center, Point(-1,-1,0), [center, obs] (const Point & p) {
    return find(obs.begin(), obs.end(), center.direction_to(p)) == obs.end();
  });
  sort(fn1.begin(), fn1.end());  
  sort(fn2.begin(), fn2.end());    
  EXPECT_EQ(fn1, fn2);

}

TEST(test_3x3, test3D) {
  // Examples from 
  //
  // Liu, S., Watterson, M., Mohta, K., Sun, K., Bhattacharya, S., Taylor, C. J., & Kumar, V. (2017). 
  // Planning dynamically feasible trajectories for quadrotors using safe flight corridors in 3-D complex environments. 
  // IEEE Robotics and Automation Letters, 2(3), 1688–1695. https://doi.org/10.1109/LRA.2017.2663526
  //
  // These examples are a bit conservative, they add more forced nn than necessary, not
  // all that are expected from the paper are returned

  Point center(0,0,0);

  // Test 3D-1: expected 
  Dir par = {-1, 0, 0};
  vector<Dir> obs = {{0, -1, 0}, {0, -1, 1}};
  vector<Dir> nn = all_neighbors(center, Point(-1,0,0), [center, obs] (const Point & p) {
    return find(obs.begin(), obs.end(), center.direction_to(p)) == obs.end();
  });
  vector<Dir> rhs = {{1,0,0}, {1,-1,0}, {1, -1, 1}};
  sort(nn.begin(), nn.end());  
  sort(rhs.begin(), rhs.end());
  EXPECT_EQ(nn, rhs);

  vector<Dir> fn1 = forced_neighbors_slow(center, Point(-1,0,0), [center, obs] (const Point & p) {
    return find(obs.begin(), obs.end(), center.direction_to(p)) == obs.end();
  });    
  vector<Dir> fn2 = forced_neighbors_fast(center, Point(-1,0,0), [center, obs] (const Point & p) {
    return find(obs.begin(), obs.end(), center.direction_to(p)) == obs.end();
  });
  sort(fn1.begin(), fn1.end());  
  sort(fn2.begin(), fn2.end());  
  EXPECT_EQ(fn1, fn2);


  // Test 3D-2: expected {{1,0,0}, {1,1,0}, {0, 1, 0}, 
  //                   {1, -1, 0}, {1,0,1}, {1,1,1}, 
  //                   {0, 1, 1}, {1, -1, 1}}

  // MISSING: {1, -1, 0}  and  {0, 1, 1} but seems fine
  par = {-1, -1, 0};
  obs = {{0, -1, 0}, {0, -1, 1}, {0, 0, 1}};
  nn = all_neighbors(center, Point(-1,-1,0), [center, obs] (const Point & p) {
    return find(obs.begin(), obs.end(), center.direction_to(p)) == obs.end();
  });

  rhs = {{1,0,0}, {1,1,0}, {0, 1, 0}, {1,0,1}, {1,1,1}, {1, -1, 1}};
  sort(nn.begin(), nn.end());  
  sort(rhs.begin(), rhs.end());
  EXPECT_EQ(nn, rhs);

  fn1 = forced_neighbors_slow(center, Point(-1,-1,0), [center, obs] (const Point & p) {
    return find(obs.begin(), obs.end(), center.direction_to(p)) == obs.end();
  });    
  fn2 = forced_neighbors_fast(center, Point(-1,-1,0), [center, obs] (const Point & p) {
    return find(obs.begin(), obs.end(), center.direction_to(p)) == obs.end();
  });
  sort(fn1.begin(), fn1.end());  
  sort(fn2.begin(), fn2.end());    
  EXPECT_EQ(fn1, fn2);

  // Test 3D-3: expected {{1,0,-1}, {1,1,-1}, 
  //                   {0, 1, -1}, {1, -1, -1},
  //                   {1,1,1}, {1,1,0}, {1,0,1}, {0,1,1},
  //                   {1,0,0}, {0,1,0}, {0,0,1}}

  // MISSING: {1, 0, -1}, {1,1,-1}, {0,1,-1}, {1,-1,-1} but seems fine
  par = {-1, -1, -1};
  obs = {{0, -1, -1}, {0, 0, -1}};
  nn = all_neighbors(center, Point(-1,-1,-1), [center, obs] (const Point & p) {
    return find(obs.begin(), obs.end(), center.direction_to(p)) == obs.end();
  });

  rhs = {{1,1,1}, {1,1,0}, {1,0,1}, {0,1,1},
                  {1,0,0}, {0,1,0}, {0,0,1}};
  sort(nn.begin(), nn.end());  
  sort(rhs.begin(), rhs.end());
  EXPECT_EQ(nn, rhs);

  fn1 = forced_neighbors_slow(center, Point(-1,-1,-1), [center, obs] (const Point & p) {
    return find(obs.begin(), obs.end(), center.direction_to(p)) == obs.end();
  });    
  fn2 = forced_neighbors_fast(center, Point(-1,-1,-1), [center, obs] (const Point & p) {
    return find(obs.begin(), obs.end(), center.direction_to(p)) == obs.end();
  });
  sort(fn1.begin(), fn1.end());  
  sort(fn2.begin(), fn2.end());  
  EXPECT_EQ(fn1, fn2);
}