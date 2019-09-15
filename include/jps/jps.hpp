#ifndef JPS_HEADER 
#define JPS_HEADER

#include <vector>
#include <queue>

#include "jps/point.hpp"
#include "jps/dir.hpp"

namespace JPS {
	typedef std::pair<JPS::Point, float> ASNode;
	
	std::pair<std::vector<Point>, float> jps(Point, Point, bool (*)(const Point &));

	std::set<Dir> neighbors(Dir, std::set<Dir>);

	// std::set<Dir> JPS::successors(Point p, Point goal);

	// Point JPS::jump(Point p, Dir d, Point goal);

}

#endif

