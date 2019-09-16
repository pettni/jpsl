#ifndef JPS_HEADER 
#define JPS_HEADER

#include <vector>
#include <queue>
#include <fstream>
#include <functional>

#include "jps/point.hpp"
#include "jps/dir.hpp"
#include "jps/encodings.hpp"

namespace JPS {

	typedef std::pair<JPS::Point, float> ASNode;

	// core JPS algos
	std::pair<std::vector<Point>, float> jps(Point, Point, const std::function<bool(const Point &)> &);
	std::set<Point> successors(const Point &, const Point &, const Point &, const std::function<bool(const Point &)> &);
	std::pair<bool, Point> jump(const Point &, const Dir &, const Point &, const std::function<bool(const Point &)> &);

	// neighbor algos
	std::set<Dir> natural_neighbors(const Point &, const Point &, const std::function<bool(const Point &)> &);
	std::set<Dir> forced_neighbors_fast(const Point &, const Point &, const std::function<bool(const Point &)> &);
	std::set<Dir> forced_neighbors_slow(const Dir &, const std::function<bool(const Dir &)> &);
	std::set<Dir> all_neighbors(const Point &, const Point &, const std::function<bool(const Point &)> &);

	// generate lookup table for forced_neighbors_fast
	void generate_lookup_table();
}

#endif