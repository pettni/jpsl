#ifndef JPSL_ENC_HEADER 
#define JPSL_ENC_HEADER

#include <functional>
#include <array>

#include "jpsl/jpsl.hpp"
#include "jpsl/dir.hpp"

namespace JPSL {

  const std::vector<Dir> NEIGHBORS_3D({{-1,-1,-1}, {1,1,1}, \
                                    {-1,-1,0}, {-1,0,-1}, {0,-1,-1}, \
                                    {-1,0,0}, {0,-1,0}, {0,0,-1}, \
                                    {-1,1,0}, {-1,0,1}, {0,-1,1}, \
                                    {1,-1,0}, {1,0,-1}, {0,1,-1}, \
                                    {-1,-1,1}, {-1,1,-1}, {1,-1,-1}, \
                                    {-1,1,1}, {1,-1,1}, {1,1,-1}, \
                                    {0,0,1}, {0,1,0}, {1,0,0}, \
                                    {0,1,1}, {1,0,1}, {1,1,0}});

  const std::vector<Dir> NIEGHBORS1D_DX({{0,1,1}, {0,1,0}, {0,1,-1}, {0,0,-1}, {0,-1,-1}, {0,-1,0}, {0,-1,1}, {0,0,1}});
  const std::vector<Dir> NIEGHBORS1D_DY({{1,0,1}, {1,0,0}, {1,0,-1}, {0,0,-1}, {-1,0,-1}, {-1,0,0}, {-1,0,1}, {0,0,1}});
  const std::vector<Dir> NIEGHBORS1D_DZ({{1,1,0}, {1,0,0}, {1,-1,0}, {0,-1,0}, {-1,-1,0}, {-1,0,0}, {-1,1,0}, {0,1,0}});

  const std::tuple<std::unordered_map<Dir, uint8_t>, 
                   std::unordered_map<Dir, uint8_t>, 
                   std::unordered_map<Dir, uint8_t>> obs_enc = {{{{0, -1, -1}, 0},
                                                                 {{0, 0, -1},  1},
                                                                 {{0, 1, -1},  2},
                                                                 {{0, -1, 0},  3},
                                                                 {{0, 1, 0},   4},
                                                                 {{0, -1, 1},  5,},
                                                                 {{0, 0, 1},   6},
                                                                 {{0, 1, 1},   7}},   

                                                                {{{-1, 0, -1}, 0},
                                                                 {{0, 0, -1},  1},
                                                                 {{0, -1, -1}, 2},
                                                                 {{-1, 0, 0},  3},
                                                                 {{0, -1, 0},  4},
                                                                 {{-1, 0, 1},  5},
                                                                 {{0, 0, 1},   6},
                                                                 {{0, -1, 1},  7}},  

                                                                {{{-1,0,-1},  0},
                                                                 {{0,0,-1},   1},
                                                                 {{0, -1,-1}, 2},
                                                                 {{-1,0,0},   3},
                                                                 {{-1,-1,0},  4},
                                                                 {{0,-1,0},   5}}};

  const std::tuple<std::unordered_map<Dir, uint16_t>, 
                   std::unordered_map<Dir, uint16_t>, 
                   std::unordered_map<Dir, uint16_t>> fnb_enc = {{{{1,-1,-1}, 0},
                                                                  {{1,0,-1},  1},
                                                                  {{1,1,-1},  2},
                                                                  {{1,-1,0},  3},
                                                                  {{1,1,0},   4},
                                                                  {{1,-1,1},  5},
                                                                  {{1,0,1},   6},
                                                                  {{1,1,1},   7}},    

                                                                 {{{-1,1,-1}, 0},
                                                                  {{0,1,-1},  1},
                                                                  {{1,1,-1},  2},
                                                                  {{1,0,-1},  3},
                                                                  {{1,-1,-1}, 4},
                                                                  {{-1,1,0},  5},
                                                                  {{1,-1,0},  6},
                                                                  {{-1,1,1},  7},
                                                                  {{0,1,1},   8},
                                                                  {{1,1,1},   9},
                                                                  {{1,0,1},  10},
                                                                  {{1,-1,1}, 11}},   

                                                                 {{{-1,1,-1}, 0},
                                                                  {{0,1,-1},  1},
                                                                  {{1,1,-1},  2},
                                                                  {{1,0,-1},  3},
                                                                  {{1,-1,-1}, 4},
                                                                  {{-1,1,0},  5},
                                                                  {{1,-1,0},  6},
                                                                  {{-1,1,1},  7},
                                                                  {{-1,0,1},  8},
                                                                  {{-1,-1,1}, 9},
                                                                  {{0,-1,1},  10},
                                                                  {{1,-1,1},  11}}};   

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


  // templates for encodings of standardized 3x3 problems
  template<int D>
  uint8_t encode_obs(const Point & node, const Point & parent, const std::function<bool(const Point &)> & state_valid) {
    Dir (*cchange)(const Dir & dir) = standardize_dir(node.direction_to(parent));
    uint8_t id = 0;

    for (auto[dir, bitkey] : std::get<D-1>(obs_enc))
      if (!state_valid(node + cchange(dir)))
        id |= 1<<bitkey;
    return id;
  }

  template<int D>
  std::vector<Dir> decode_obs(const uint8_t & id){
    std::vector<Dir> ret;
    for (auto[dir, bitkey] : std::get<D-1>(obs_enc))
      if (id & (1<<bitkey))
        ret.push_back(dir);
    return std::move(ret);
  }


  template<int D>
  uint16_t encode_fn(const std::vector<Dir> & fn_set) {
    uint16_t id = 0;
    for (const Dir & d : fn_set)
      id |= 1<<(std::get<D-1>(fnb_enc).at(d));
    return id;
  }

  template<int D>
  std::vector<Dir> decode_fn(const Point & node, const Point & parent, const uint16_t & id) {
    Dir (*cchange)(const Dir & dir) = standardize_dir(node.direction_to(parent));

    std::vector<Dir> ret;
    for (auto[dir, bitkey] : std::get<D-1>(fnb_enc))
      if (id & (1<<bitkey))
          ret.push_back(cchange(dir));
    return std::move(ret);
  }

}

#endif
