#include "jpsl/jpsl.hpp"

using namespace std;
using namespace JPSL;

Point::Point(int64_t x, int64_t y, int64_t z) : x_(x), y_(y), z_(z) {}

Point Point::operator+(const Dir & d) const {
  return move(Point(x_ + d.dx_, y_ + d.dy_, z_ + d.dz_));
}

Point & Point::operator+=(const Dir & d) {
  x_ += d.dx_;
  y_ += d.dy_;
  z_ += d.dz_;
  return *this;
}

Point Point::operator-(const Dir & d) const {
  return move(Point(x_ - d.dx_, y_ - d.dy_, z_ - d.dz_));
}

Point Point::operator-(const Point & p) const {
  return move(Point(x_ - p.x_, y_ - p.y_, z_ - p.z_));
}

bool Point::operator==(const Point & p) const {
  return make_tuple(x_,y_,z_) == make_tuple(p.x_, p.y_, p.z_);
}

bool Point::operator!=(const Point & p) const {
  return make_tuple(x_,y_,z_) != make_tuple(p.x_, p.y_, p.z_);
}

bool Point::operator<(const Point & p) const {
  return make_tuple(x_,y_,z_) < make_tuple(p.x_, p.y_, p.z_);
}

float Point::norm() const {
  return sqrt(float(x_*x_ + y_*y_ + z_*z_));
}

float Point::manhattan_norm() const {
  return abs(x_) + abs(y_) + abs(z_);
}

Dir Point::direction_to(const Point & p) const {
  int64_t distance = max(abs(p.x_-x_), max(abs(p.y_-y_), abs(p.z_-z_)));
  if (distance == 0)
    return move(Dir(0,0,0));
  return move(Dir((p.x_-x_)/distance, (p.y_-y_)/distance, (p.z_-z_)/distance));
}