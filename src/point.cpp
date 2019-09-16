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

Dir Point::incoming_dir(const Point & p) const {
	int64_t dx = p.x_ - x_;
	int64_t dy = p.y_ - y_;
	int64_t dz = p.z_ - z_;

	uint8_t order = uint8_t(dx != 0) + uint8_t(dy != 0) + uint8_t(dz != 0);

	int64_t distance = (abs(dx) + abs(dy) + abs(dz)) / order;

	if (dx%distance != 0 || dy%distance != 0 || dz%distance != 0)
		throw std::string("difference is not valid jump");

	return Dir(dx/distance, dy/distance, dz/distance);
}