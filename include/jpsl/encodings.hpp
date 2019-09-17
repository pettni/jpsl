#ifndef JPSL_ENC_HEADER 
#define JPSL_ENC_HEADER

#include <functional>
#include <set>

#include "jpsl/jpsl.hpp"
#include "jpsl/dir.hpp"

namespace JPSL {

  const std::set<Dir> NEIGHBORS_3D({{-1,-1,-1}, {1,1,1}, \
                                    {-1,-1,0}, {-1,0,-1}, {0,-1,-1}, \
                                    {-1,0,0}, {0,-1,0}, {0,0,-1}, \
                                    {-1,1,0}, {-1,0,1}, {0,-1,1}, \
                                    {1,-1,0}, {1,0,-1}, {0,1,-1}, \
                                    {-1,-1,1}, {-1,1,-1}, {1,-1,-1}, \
                                    {-1,1,1}, {1,-1,1}, {1,1,-1}, \
                                    {0,0,1}, {0,1,0}, {1,0,0}, \
                                    {0,1,1}, {1,0,1}, {1,1,0}});

  const std::set<Dir> NIEGHBORS1D_DX({{0,1,1}, {0,1,0}, {0,1,-1}, {0,0,-1}, {0,-1,-1}, {0,-1,0}, {0,-1,1}, {0,0,1}});
  const std::set<Dir> NIEGHBORS1D_DY({{1,0,1}, {1,0,0}, {1,0,-1}, {0,0,-1}, {-1,0,-1}, {-1,0,0}, {-1,0,1}, {0,0,1}});
  const std::set<Dir> NIEGHBORS1D_DZ({{1,1,0}, {1,0,0}, {1,-1,0}, {0,-1,0}, {-1,-1,0}, {-1,0,0}, {-1,1,0}, {0,1,0}});

  extern uint8_t const lookup1d[256];
  extern uint16_t const lookup2d[256];
  extern uint16_t const lookup3d[64];

  // pre-defined coordinate changes
  Dir fnd_id(const Dir &);
  Dir fnd_neg(const Dir &);
  Dir fnd_fp_x(const Dir &);
  Dir fnd_fp_y(const Dir &);
  Dir fnd_fp_z(const Dir &);
  Dir fnd_fp_xz(const Dir &);
  Dir fnd_fp_xy(const Dir &);
  Dir fnd_fp_yz(const Dir &);
  Dir fnd_ch_xy(const Dir &);
  Dir fnd_ch_xz(const Dir &);
  Dir fnd_ch_yz(const Dir &);
  Dir fnd_chfp_xy(const Dir &);
  Dir fnd_chfp_xz(const Dir &);
  Dir fnd_chfp_yz(const Dir &);
  Dir fnd_fp_x_ch_yz(const Dir &);
  Dir fnd_fp_y_ch_xz(const Dir &);
  Dir fnd_fp_x_chfp_yz(const Dir &);
  Dir fnd_fp_y_chfp_xz(const Dir &);

  // coordinate change to standard 3x3 problem
  Dir(*standardize_dir(const Dir &))(const Dir &);

  // encoding and decoding of 3x3 problems
  uint8_t encode_obs_1d(const Point &, const Point &, const std::function<bool(const Point &)> &);
  uint8_t encode_obs_2d(const Point &, const Point &, const std::function<bool(const Point &)> &);
  uint8_t encode_obs_3d(const Point &, const Point &, const std::function<bool(const Point &)> &);

  std::set<Dir> decode_fn_1d(const Point &, const Point &, const uint8_t &);
  std::set<Dir> decode_fn_2d(const Point &, const Point &, const uint16_t &);
  std::set<Dir> decode_fn_3d(const Point &, const Point &, const uint16_t &);

  // inverse encoding and decoding for lookup table generation
  std::set<Dir> decode_obs_1d(const uint8_t &);
  std::set<Dir> decode_obs_2d(const uint8_t &);
  std::set<Dir> decode_obs_3d(const uint8_t &);

  uint8_t encode_fn_1d(const std::set<Dir> &);
  uint16_t encode_fn_2d(const std::set<Dir> &);
  uint16_t encode_fn_3d(const std::set<Dir> &);

}

#endif
