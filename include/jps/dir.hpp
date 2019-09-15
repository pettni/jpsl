#ifndef JPS_DIR_HEADER 
#define JPS_DIR_HEADER

#include <stdint.h>
#include <iostream>
#include <math.h>

namespace JPS {

  class Dir {
  public:
    Dir(uint8_t);
    operator int8_t() const;

    Dir(int8_t dx, int8_t dy, int8_t dz);

    int8_t dx() const;
    int8_t dy() const;
    int8_t dz() const;

    uint8_t order() const;

    Dir operator-() const;
    float distance_to(const Dir &) const;
    float norm() const;

  private:
    uint8_t data;
  };

  inline std::ostream& operator<<(std::ostream& os, const JPS::Dir & d) {
    os << "[" << (int) d.dx() << "," << (int) d.dy() << "," << (int) d.dz() << "]";
    return os;
  }

}


#endif