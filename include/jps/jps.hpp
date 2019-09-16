#ifndef JPS_HEADER 
#define JPS_HEADER

#include <vector>
#include <queue>

#include "jps/point.hpp"
#include "jps/dir.hpp"


namespace JPS {

	const std::set<Dir> NEIGHBORS_3D({{-1,-1,-1}, {1,1,1}, \
																    {-1,-1,0}, {-1,0,-1}, {0,-1,-1}, \
																    {-1,0,0}, {0,-1,0}, {0,0,-1}, \
																    {-1,1,0}, {-1,0,1}, {0,-1,1}, \
																    {1,-1,0}, {1,0,-1}, {0,1,-1}, \
																    {-1,-1,1}, {-1,1,-1}, {1,-1,-1}, \
																    {-1,1,1}, {1,-1,1}, {1,1,-1}, \
																    {0,0,1}, {0,1,0}, {1,0,0}, \
																    {0,1,1}, {1,0,1}, {1,1,0}});

	typedef std::pair<JPS::Point, float> ASNode;

	std::pair<std::vector<Point>, float> jps(Point, Point, bool (*)(const Point &));

	std::pair<bool, Point> jump(const Point &, const Dir &, const Point &, bool (*)(const Point &));

	std::set<Point> successors(const Point &, const Point &, const Point &, bool (*)(const Point &));

	std::set<Dir> natural_neighbors(const Point &, const Point &, bool (*)(const Point &));
	std::set<Dir> forced_neighbors(const Point &, const Point &, bool (*)(const Point &));
	std::set<Dir> all_neighbors(const Point &, const Point &, bool (*)(const Point &));

	std::set<Dir> natural_neighbors_(const Dir &, const std::set<Dir> &);
	std::set<Dir> forced_neighbors_(const Dir &, const std::set<Dir> &);
	std::set<Dir> all_neighbors_(const Dir &, const std::set<Dir> &);
}

#endif

