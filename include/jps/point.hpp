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

	  Point operator+(const Dir &) const;
	  Point operator-(const Point &) const;
	  bool operator==(const Point &) const;
	  bool operator!=(const Point &) const;
	  bool operator<(const Point &) const;
	  float norm() const;

	  friend std::ostream& operator<<(std::ostream&, const JPS::Point&);

	private:
	  int64_t x, y, z;
	};

	inline std::ostream& operator<<(std::ostream& os, const JPS::Point& p) {
	    os << "[" << (int) p.x << "," << (int) p.y << "," << (int) p.z << "]";
	    return os;
	}

}

#endif