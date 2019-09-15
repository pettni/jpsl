#include <iostream>

#include "jps/jps.hpp"

using namespace JPS;
using namespace std;

bool state_valid(long x, long y, long z) {
  return x >= 0 && x < 10 && y >= 0 && y < 10 && z >= 0 && z < 10;
}

int main(int argc, char const *argv[]) {

  auto[path, cost] = jps({0,0,0}, {7,4,6});

  cout << "Solved with cost " << cost << endl;

  for (Point p : path)
  	cout << p << endl;

  return 0;
}