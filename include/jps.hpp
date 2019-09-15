#include <stdint.h>
#include <iostream>
#include <math.h>
#include <vector>
#include <set>
#include <map>
#include <algorithm>

#ifndef JPS_HEADER 
#define JPS_HEADER

using namespace std;

class Dir {
public:
  Dir(uint8_t);
  operator int8_t() const;

  Dir(int8_t dx, int8_t dy, int8_t dz);

  int8_t dx() const;
  int8_t dy() const;
  int8_t dz() const;

  uint8_t order() const;

  Dir operator-();
  float distance_to(const Dir &) const;
  float norm() const;

private:
  uint8_t data;
};

ostream& operator<<(ostream& stream, const Dir& dir);

set<Dir> expand(Dir parent, set<Dir> obstacles);

#endif