#include "jps/point.hpp"

using namespace JPS;

Point::Point(int64_t x, int64_t y, int64_t z) : x_(x), y_(y), z_(z) {}

Point Point::operator+(const Dir & d) const {
	return Point(x_ + d.dx(), y_ + d.dy(), z_ + d.dz());
}

Point Point::operator-(const Point & p) const {
	return Point(x_ - p.x_, y_ - p.y_, z_ - p.z_);
}

bool Point::operator==(const Point & p) const {
	return std::make_tuple(x_,y_,z_) == std::make_tuple(p.x_, p.y_, p.z_);
}

bool Point::operator!=(const Point & p) const {
	return std::make_tuple(x_,y_,z_) != std::make_tuple(p.x_, p.y_, p.z_);
}

bool Point::operator<(const Point & p) const {
	return std::make_tuple(x_,y_,z_) < std::make_tuple(p.x_, p.y_, p.z_);
}

float Point::norm() const {
	return sqrt(float(x_*x_ + y_*y_ + z_*z_));
}
