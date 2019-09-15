#include "jps.hpp"

#include <iostream>

int main(int argc, char const *argv[]) {

  // Test 2D-1: expected {1,0,0}
  Dir par = {-1, 0, 0};
  set<Dir> obs = {};
  set<Dir> neighbors = expand(par, obs);

  cout << "Test 2D-1" << endl;
  for (Dir d : neighbors)
    cout << d << endl;

  // Test 2D-2:  expected {{1,0,0}, {1,1,0}}
  par = {-1, 0, 0};
  obs = {{0,1,0}};
  neighbors = expand(par, obs);

  cout << "Test 2D-2" << endl;
  for (Dir d : neighbors)
    cout << d << endl;

  // Test 2D-3:  expected {{1,0,0}, {1,1,0}, {0,1,0}}
  par = {-1, -1, 0};
  obs = {};
  neighbors = expand(par, obs);

  cout << "Test 2D-3" << endl;
  for (Dir d : neighbors)
    cout << d << endl;

  // Test 2D-4:  expected {{-1,1,0}, {1,0,0}, {1,1,0}, {0,1,0}}
  par = {-1, -1, 0};
  obs = {{-1,0,0}, {-1,0,1}, {-1,0,-1}};
  neighbors = expand(par, obs);

  cout << "Test 2D-4" << endl;
  for (Dir d : neighbors)
    cout << d << endl;

  // Test 3D-1: expected {{1,0,0}, {1,-1,0}, {1, -1, 1}}
  par = {-1, 0, 0};
  obs = {{0, -1, 0}, {0, -1, 1}};
  neighbors = expand(par, obs);

  cout << "Test 3D-1" << endl;
  for (Dir d : neighbors)
    cout << d << endl;

  // Test 3D-2: expected {{1,0,0}, {1,1,0}, {0, 1, 0}, 
  //                   {1, -1, 0}, {1,0,1}, {1,1,1}, 
  //                   {0, 1, 1}, {1, -1, 1}}

  // MISSING: {1, -1, 0}  and  {0, 1, 1} but seems fine
  par = {-1, -1, 0};
  obs = {{0, -1, 0}, {0, -1, 1}, {0, 0, 1}};
  neighbors = expand(par, obs);

  cout << "Test 3D-2" << endl;
  for (Dir d : neighbors)
    cout << d << endl;

  // Test 3D-3: expected {{1,0,-1}, {1,1,-1}, 
  //                   {0, 1, -1}, {1, -1, -1},
  //                   {1,1,1}, {1,1,0}, {1,0,1}, {0,1,1},
  //                   {1,0,0}, {0,1,0}, {0,0,1}}

  // MISSING: {1, 0, -1}, {1,1,-1}, {0,1,-1}, {1,-1,-1} but seems fine
  par = {-1, -1, -1};
  obs = {{0, -1, -1}, {0, 0, -1}};
  neighbors = expand(par, obs);

  cout << "Test 3D-3" << endl;
  for (Dir d : neighbors)
    cout << d << endl;


  return 0;
}