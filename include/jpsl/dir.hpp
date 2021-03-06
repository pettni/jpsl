#ifndef JPSL_DIR_HEADER 
#define JPSL_DIR_HEADER

#include <stdint.h>
#include <iostream>
#include <math.h>

namespace JPSL {

  class Dir {
  public:
    Dir(uint8_t);
    operator uint8_t() const;

    Dir(int8_t dx, int8_t dy, int8_t dz);

    int8_t dx() const;
    int8_t dy() const;
    int8_t dz() const;

    uint8_t order() const;

    Dir operator-() const;
    float distance_to(const Dir &) const;
    float norm() const;

    friend class Point;
  private:
    int8_t dx_, dy_, dz_;
  };

  inline std::ostream& operator<<(std::ostream& os, const JPSL::Dir & d) {
    os << "[" << (int) d.dx() << "," << (int) d.dy() << "," << (int) d.dz() << "]";
    return os;
  }
}

namespace std {
  template <>
  struct hash<JPSL::Dir> {
    size_t operator()(const JPSL::Dir & d) const {
      return hash<uint8_t>()(d);
    }
  };
}

#endif