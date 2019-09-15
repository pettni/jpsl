#include "jps/point.hpp"

using namespace JPS;

Point::Point(int64_t x, int64_t y, int64_t z) : x(x), y(y), z(z) {}

Point Point::operator+(const Dir & d) const {
	return Point(x + d.dx(), y + d.dy(), z + d.dz());
}

Point Point::operator-(const Point & p) const {
	return Point(x - p.x, y - p.y, z - p.z);
}

bool Point::operator==(const Point & p) const {
	return std::make_tuple(x,y,z) == std::make_tuple(p.x, p.y, p.z);
}

bool Point::operator!=(const Point & p) const {
	return std::make_tuple(x,y,z) != std::make_tuple(p.x, p.y, p.z);
}

bool Point::operator<(const Point & p) const {
	return std::make_tuple(x,y,z) < std::make_tuple(p.x, p.y, p.z);
}

float Point::norm() const {
	return sqrt(float(x*x + y*y + z*z));
}
