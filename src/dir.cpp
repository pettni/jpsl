#include "jps/dir.hpp"

using namespace std;
using namespace JPS;

Dir::Dir(uint8_t data) : data(data) {}

Dir::operator int8_t() const {return data;}

Dir::Dir(int8_t dx, int8_t dy, int8_t dz) : data(dx+1 + 3*(dy+1) + 9*(dz+1)) {}

int8_t Dir::dx() const {return data % 3 - 1;}

int8_t Dir::dy() const {return (data/3)%3 - 1;}

int8_t Dir::dz() const {return (data/9)%3 - 1;}

uint8_t Dir::order() const {
  return uint8_t((data % 3) != 1) + uint8_t(((data/3)%3) != 1) + uint8_t(((data/9)%3) != 1);
}

Dir Dir::operator-() const {
  return Dir(-dx(),-dy(),-dz());
}

float Dir::distance_to(const Dir & other) const {
  return sqrt(float((dx() - other.dx())*(dx() - other.dx()) + 
                    (dy() - other.dy())*(dy() - other.dy()) +
                    (dz() - other.dz())*(dz() - other.dz())));
}

float Dir::norm() const {
  return sqrt(float(dx()*dx() + dy()*dy() + dz()*dz()));
}