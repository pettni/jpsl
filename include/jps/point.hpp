#ifndef JPS_POINT_HEADER 
#define JPS_POINT_HEADER

#include <stdint.h>
#include <iostream>
#include <math.h>
#include <vector>
#include <set>
#include <map>
#include <algorithm>

#include "jps/dir.hpp"

namespace JPS {

	class Point {
	public:
	  Point(int64_t, int64_t, int64_t);
	  Point(const Point &) = default;
	  Point& operator=(const Point &) = default;

	  inline int64_t x() const {return x_;}
	  inline int64_t y() const {return y_;}
	  inline int64_t z() const {return z_;}

	  Point operator+(const Dir &) const;
	  Point operator-(const Point &) const;
	  bool operator==(const Point &) const;
	  bool operator!=(const Point &) const;
	  bool operator<(const Point &) const;
	  float norm() const;

	private:
	  int64_t x_, y_, z_;
	};

	inline std::ostream& operator<<(std::ostream& os, const JPS::Point& p) {
	    os << "[" << (int) p.x() << "," << (int) p.y() << "," << (int) p.z() << "]";
	    return os;
	}

}

#endif