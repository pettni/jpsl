#ifndef JPS_HEADER 
#define JPS_HEADER

#include <vector>
#include <queue>
#include <functional>
#include <map>
#include <set>

#include "jps/point.hpp"
#include "jps/dir.hpp"
#include "jps/encodings.hpp"

namespace JPS {

  typedef std::tuple<JPS::Point, JPS::Point, float> ASNode;   // node, parent, running distance

  // core JPS algos
  std::pair<std::vector<Point>, float> jps(Point, Point, const std::function<bool(const Point &)> &);
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