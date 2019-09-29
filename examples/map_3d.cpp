#include <fstream>
#include <string>
#include <functional>
#include <chrono>

#include "jpsl/jpsl.hpp"

using namespace std;
using namespace JPSL;

bool is_valid(const Point & p, const vector<bool> & obs, 
              const uint64_t & xsize, const uint64_t & ysize, const uint64_t & zsize) {

  if (p.x() < 0 || p.y() < 0 || p.z() < 0 || uint64_t(p.x()) >= xsize || uint64_t(p.y()) >= ysize || uint64_t(p.z()) >= zsize)
    return false;
  return !obs[p.x() + p.y() * xsize + p.z() * xsize * ysize];
}

int main(int argc, char const *argv[]) {
  ifstream infile;

  if (argc < 9) {
    cout << "Usage: map_3d mapfile max_jump x0 y0 z0 x1 y1 z3" << endl;
    return 0;
  }

  infile.open(argv[1]);
  int max_jump = atoi(argv[2]);

  string s;
  uint64_t x, y, z, xsize, ysize, zsize;

  infile >> s >> xsize >> ysize >> zsize;

  vector<bool> obstacles(xsize * ysize * zsize, false);

  while (infile >> x >> y >> z)
    obstacles[x + y * xsize + z * xsize * ysize] = true;

  infile.close();

  Point start(atoi(argv[3]),atoi(argv[4]),atoi(argv[5]));
  Point goal(atoi(argv[6]),atoi(argv[7]),atoi(argv[8]));

  if (!is_valid(start, obstacles, xsize, ysize, zsize)){
    cout << "Start state not valid" << endl;
    return 0;
  }

  if (!is_valid(goal, obstacles, xsize, ysize, zsize)) {
    cout << "Goal state not valid" << endl;
    return 0;
  }

  cout << "Finding path from " << start << " to " << goal << " with max_jump=" << max_jump << endl;

  auto t0 = chrono::steady_clock::now();

  auto[path, length] = plan(start, goal, max_jump, std::bind(is_valid, placeholders::_1, obstacles, xsize, ysize, zsize));

  auto t1 = chrono::steady_clock::now();

  cout << "Found path with length " << length << " in " << chrono::duration_cast<chrono::milliseconds>(t1 - t0).count() << "ms" << endl;
  for (Point p : path) 
    cout << p << endl;

  return 0;
}