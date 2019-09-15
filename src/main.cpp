#include <iostream>

#include "jps/jps.hpp"

using namespace JPS;
using namespace std;

bool state_valid(const Point & p) {


  if (p.x() < 0 || p.x() >= 8 || p.y() < 0 || p.y() >= 5 || p.z() < 0 || p.z() > 0)
  	return false;

  if (p.x() == 5 && p.y() < 3)
  	return false;

  return true;
}

int main(int argc, char const *argv[]) {

  auto[path, cost] = jps({0,0,0}, {7,0,0}, &state_valid);

  cout << "Solved with cost " << cost << endl;

  for (Point p : path)
  	cout << p << endl;

  return 0;
}