#ifndef JPSL_HEADER 
#define JPSL_HEADER

#include <math.h>

#include <vector>
#include <queue>
#include <functional>
#include <map>
#include <unordered_map>
#include <unordered_set>
#include <stdint.h>
#include <iostream>
#include <tuple>

#include "jpsl/dir.hpp"
#include "jpsl/point.hpp"

namespace JPSL {

  typedef std::tuple<JPSL::Point, JPSL::Point, float> ASNode;   // node, parent, running distance

  // core JPS algos
  std::pair<std::vector<Point>, float> plan(Point, Point, const std::function<bool(const Point &)> &);
  std::pair<bool, Point> jump(const Point &, const Dir &, const Point &, const std::function<bool(const Point &)> &);

  // neighbor algos
  std::unordered_set<Dir> natural_neighbors(const Point &, const Point &, const std::function<bool(const Point &)> &);
  std::unordered_set<Dir> forced_neighbors_fast(const Point &, const Point &, const std::function<bool(const Point &)> &);
  std::unordered_set<Dir> forced_neighbors_slow(const Point &, const Point &, const std::function<bool(const Point &)> &);
  std::unordered_set<Dir> all_neighbors(const Point &, const Point &, const std::function<bool(const Point &)> &);

  bool has_forced_neighbor(const Point &, const Point &, const std::function<bool(const Point &)> &);
  
  std::unordered_set<Dir> forced_neighbors_fast_1d(const Point &, const Point &, const std::function<bool(const Point &)> &);
  bool has_forced_neighbor_fast_1d(const Point &, const Point &, const std::function<bool(const Point &)> &);

  // iterator class for JPS successors
  class JPSLSucc {
    public:

    class JPSLSucc_iter {
    public:
    JPSLSucc_iter(const Point &, const Point &, const std::function<bool(const Point &)> &, const std::unordered_set<Dir> &);
    JPSLSucc_iter(const Point &, const Point &, const std::function<bool(const Point &)> &, const std::unordered_set<Dir> &, std::unordered_set<Dir>::iterator);    
      typedef Dir value_type;
      JPSLSucc_iter & operator++();
      Point operator*();
      bool operator!=(const JPSLSucc_iter &);

      friend class JPSLSucc;
    private:
      const Point & node;
      const Point & goal;
      const std::function<bool(const Point &)> & state_valid;
      const std::unordered_set<Dir> & jump_dirs;
      std::pair<bool, Point> res;
      std::unordered_set<Dir>::const_iterator it;
    };

    JPSLSucc(const Point &, const Point &, const Point &, const std::function<bool(const Point &)> &);
    JPSLSucc_iter begin();
    JPSLSucc_iter end();

    private:
    const Point & node;
    const Point & goal;
    const std::function<bool(const Point &)> & state_valid;
    std::unordered_set<Dir> jump_dirs;
  };

}

#endif