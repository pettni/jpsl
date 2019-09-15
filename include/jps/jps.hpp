#ifndef JPS_HEADER 
#define JPS_HEADER

#include <vector>
#include <queue>

#include "jps/point.hpp"
#include "jps/dir.hpp"

namespace JPS {
	typedef std::pair<JPS::Point, float> ASNode;
	
	std::pair<std::vector<Point>, float> jps(Point, Point);

	std::set<Dir> expand(Dir, std::set<Dir>);
}

#endif

