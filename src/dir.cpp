#include "jpsl/dir.hpp"

using namespace std;
using namespace JPSL;

Dir::Dir(uint8_t data) : dx_(data % 3 - 1), dy_((data/3)%3 - 1), dz_((data/9)%3 - 1) {}

Dir::operator uint8_t() const {return dx_+1 + 3*(dy_+1) + 9*(dz_+1);}

Dir::Dir(int8_t dx, int8_t dy, int8_t dz) : dx_(dx), dy_(dy), dz_(dz) {}

int8_t Dir::dx() const {return dx_;}

int8_t Dir::dy() const {return dy_;}

int8_t Dir::dz() const {return dz_;}

uint8_t Dir::order() const {
  return uint8_t(dx_!=0) + uint8_t(dy_!=0) + uint8_t(dz_!=0);
}

Dir Dir::operator-() const {
  return move(Dir(-dx_,-dy_,-dz_));
}

float Dir::distance_to(const Dir & other) const {
  return sqrt(float((dx_ - other.dx_)*(dx_ - other.dx_) + 
                    (dy_ - other.dy_)*(dy_ - other.dy_) +
                    (dz_ - other.dz_)*(dz_ - other.dz_)));
}

float Dir::norm() const {
  return sqrt(float(dx_*dx_ + dy_*dy_ + dz_*dz_));
}