#ifndef JPSL_HEADER 
#define JPSL_HEADER

#include <math.h>

#include <vector>
#include <queue>
#include <functional>
#include <map>
#include <unordered_map>
#include <set>
#include <stdint.h>
#include <iostream>
#include <tuple>

#include "jpsl/dir.hpp"
#include "jpsl/point.hpp"

namespace JPSL {

  typedef std::tuple<JPSL::Point, JPSL::Point, float> ASNode;   // node, parent, running distance

  // core JPS algos
  std::pair<std::vector<Point>, float> plan(Point, Point, const std::function<bool(const Point &)> &);
  std::set<Point> successors(const Point &, const Point &, const Point &, const std::function<bool(const Point &)> &);
  std::pair<bool, Point> jump(const Point &, const Dir &, const Point &, const std::function<bool(const Point &)> &);

  // neighbor algos
  std::set<Dir> natural_neighbors(const Point &, const Point &, const std::function<bool(const Point &)> &);
  std::set<Dir> forced_neighbors_fast(const Point &, const Point &, const std::function<bool(const Point &)> &);
  std::set<Dir> forced_neighbors_slow(const Point &, const Point &, const std::function<bool(const Point &)> &);
  std::set<Dir> all_neighbors(const Point &, const Point &, const std::function<bool(const Point &)> &);

  bool has_forced_neighbor(const Point &, const Point &, const std::function<bool(const Point &)> &);
  
  std::set<Dir> forced_neighbors_fast_1d(const Point &, const Point &, const std::function<bool(const Point &)> &);
  bool has_forced_neighbor_fast_1d(const Point &, const Point &, const std::function<bool(const Point &)> &);
}

#endif