#ifndef JPS_HEADER 
#define JPS_HEADER

#include <vector>
#include <queue>

#include "jps/point.hpp"
#include "jps/dir.hpp"

namespace JPS {
	typedef std::pair<JPS::Point, float> ASNode;
	
	std::pair<std::vector<Point>, float> jps(Point, Point, bool (*)(const Point &));

	std::set<Dir> all_neighbors(const Point &, const Point &, bool (*)(const Point &));
	std::set<Dir> natural_neighbors(const Point &, const Point &, bool (*)(const Point &));
	std::set<Dir> forced_neighbors(const Point &, const Point &, bool (*)(const Point &));

	std::set<Dir> all_neighbors(const Dir &, const std::set<Dir> &);
	std::set<Dir> natural_neighbors(const Dir &, const std::set<Dir> &);
	std::set<Dir> forced_neighbors(const Dir &, const std::set<Dir> &);

	// std::set<Dir> JPS::successors(Point p, Point goal);

	std::pair<bool, Point> jump(const Point &, const Dir &, const Point &, bool (*)(const Point &));

}

#endif

