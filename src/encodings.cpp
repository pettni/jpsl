#include "jps/jps.hpp"

using namespace std;
using namespace JPS;

// 1D coordinate changes
Dir JPS::fnd_id(const Dir & dir) { return move(dir); }
Dir JPS::fnd_neg(const Dir & dir) { return move(-dir); }
Dir JPS::fnd_fp_x(const Dir & dir) { return move(Dir(-dir.dx(), dir.dy(), dir.dz())); }
Dir JPS::fnd_fp_y(const Dir & dir) { return move(Dir(dir.dx(), -dir.dy(), dir.dz())); }
Dir JPS::fnd_fp_z(const Dir & dir) { return move(Dir(dir.dx(), dir.dy(), -dir.dz())); }
Dir JPS::fnd_fp_xz(const Dir & dir) { return move(Dir(-dir.dx(), dir.dy(), -dir.dz())); }
Dir JPS::fnd_fp_xy(const Dir & dir) { return move(Dir(-dir.dx(), -dir.dy(), dir.dz())); }
Dir JPS::fnd_fp_yz(const Dir & dir) { return move(Dir(dir.dx(), -dir.dy(), -dir.dz())); }
Dir JPS::fnd_ch_xy(const Dir & dir) { return move(Dir(dir.dy(), dir.dx(), dir.dz())); }
Dir JPS::fnd_ch_xz(const Dir & dir) { return move(Dir(dir.dz(), dir.dy(), dir.dx())); }
Dir JPS::fnd_ch_yz(const Dir & dir) { return move(Dir(dir.dx(), dir.dz(), dir.dy())); }
Dir JPS::fnd_chfp_xy(const Dir & dir) { return move(Dir(-dir.dy(), -dir.dx(), dir.dz())); }
Dir JPS::fnd_chfp_xz(const Dir & dir) { return move(Dir(-dir.dz(), dir.dy(), -dir.dx())); }
Dir JPS::fnd_chfp_yz(const Dir & dir) { return move(Dir(dir.dx(), -dir.dz(), -dir.dy())); }
Dir JPS::fnd_fp_x_ch_yz(const Dir & dir) { return move(Dir(-dir.dx(), dir.dz(), dir.dy())); }
Dir JPS::fnd_fp_y_ch_xz(const Dir & dir) { return move(Dir(dir.dz(), -dir.dy(), dir.dx())); }
Dir JPS::fnd_fp_x_chfp_yz(const Dir & dir) { return move(Dir(-dir.dx(), -dir.dz(), -dir.dy())); }
Dir JPS::fnd_fp_y_chfp_xz(const Dir & dir) { return move(Dir(-dir.dz(), -dir.dy(), -dir.dx())); }

Dir(*JPS::standardize_dir(const Dir & d0))(const Dir &) {
  // coordinate change so that d0 becomes one of [-1 0 0] [-1 -1 0] or [-1 -1 -1]
  // and s.t. coordinate change is its own inverse

  switch(d0.order()) {
    case 1:
      if (d0.dx() == -1)  // identity 
        return &fnd_id;
      if (d0.dx() == 1)   // mirror
        return &fnd_neg;
      if (d0.dy() == -1)  // change x,y
        return &fnd_ch_xy;
      if (d0.dy() == 1)   // change-flip x,y
        return &fnd_chfp_xy;
      if (d0.dz() == -1)  // change x,z
        return &fnd_ch_xz;
      if (d0.dz() == 1)   // change-flip x,z
        return &fnd_chfp_xz;
      break;
    case 2:
      if (d0.dx() == -1 && d0.dy() == -1) // identity 
        return &fnd_id;
      if (d0.dx() == 1 && d0.dy() == 1)   // flip all
        return &fnd_neg;
      if (d0.dx() == 1 && d0.dy() == -1)  // flip x
        return &fnd_fp_x;
      if (d0.dx() == -1 && d0.dy() == 1)  // flip y
        return &fnd_fp_y;

      if (d0.dx() == -1 && d0.dz() == -1) // change y/z 
        return &fnd_ch_yz;
      if (d0.dx() == -1 && d0.dz() == 1)  // change-flip y/z
        return &fnd_chfp_yz;
      if (d0.dx() == 1 && d0.dz() == 1)   // flip x, change-flip y/z
        return &fnd_fp_x_chfp_yz;
      if (d0.dx() == 1 && d0.dz() == -1)  // flip x, change y/z
        return &fnd_fp_x_ch_yz;

      if (d0.dy() == -1 && d0.dz() == -1)  // change x/z 
        return &fnd_ch_xz;
      if (d0.dy() == -1 && d0.dz() == 1)   // change-flip x/z 
        return &fnd_chfp_xz;
      if (d0.dy() == 1 && d0.dz() == 1)    // flip y, change-flip x/z
        return &fnd_fp_y_chfp_xz;
      if (d0.dy() == 1 && d0.dz() == -1)   // flip y, change x/z
        return &fnd_fp_y_ch_xz;
      break;
    case 3:
      if (d0.dx() == -1 && d0.dy() == -1 && d0.dz() == -1)  // identity 
        return &fnd_id;
      if (d0.dx() == 1 && d0.dy() == 1 && d0.dz() == 1)     // flip all 
        return &fnd_neg;

      if (d0.dx() == 1 && d0.dy() == 1 && d0.dz() == -1)  // flip x,y 
        return &fnd_fp_xy;
      if (d0.dx() == 1 && d0.dy() == -1 && d0.dz() == 1)  // flip x,z 
        return &fnd_fp_xz;
      if (d0.dx() == -1 && d0.dy() == 1 && d0.dz() == 1)  // flip y,z
        return &fnd_fp_yz;

      if (d0.dx() == 1 && d0.dy() == -1 && d0.dz() == -1)  // flip x
        return &fnd_fp_x;
      if (d0.dx() == -1 && d0.dy() == 1 && d0.dz() == -1)  // flip y
        return &fnd_fp_y;
      if (d0.dx() == -1 && d0.dy() == -1 && d0.dz() == 1)  // flip z
        return &fnd_fp_z;
      break;
  }

  return &fnd_id;
}

uint8_t JPS::encode_obs_1d(const Point & node, const Point & parent, const std::function<bool(const Point &)> & state_valid) {
  Dir (*cchange)(const Dir & dir) = standardize_dir(node.direction_to(parent));

  uint8_t ret = 0;
  if (!state_valid(node + cchange(Dir(0, -1, -1))))
    ret |= 1;
  if (!state_valid(node + cchange(Dir(0, 0, -1))))
    ret |= 1<<1;
  if (!state_valid(node + cchange(Dir(0, 1, -1))))
    ret |= 1<<2;
  if (!state_valid(node + cchange(Dir(0, -1, 0))))
    ret |= 1<<3;
  if (!state_valid(node + cchange(Dir(0, 1, 0))))
    ret |= 1<<4;
  if (!state_valid(node + cchange(Dir(0, -1, 1))))
    ret |= 1<<5;
  if (!state_valid(node + cchange(Dir(0, 0, 1))))
    ret |= 1<<6;
  if (!state_valid(node + cchange(Dir(0, 1, 1))))
    ret |= 1<<7;
  return ret;
}

uint8_t JPS::encode_obs_2d(const Point & node, const Point & parent, const std::function<bool(const Point &)> & state_valid) {
  Dir (*cchange)(const Dir & dir) = standardize_dir(node.direction_to(parent));

  uint8_t ret = 0;
  if (!state_valid(node + cchange(Dir(-1, 0, -1))))
    ret |= 1;
  if (!state_valid(node + cchange(Dir(0, 0, -1))))
    ret |= 1<<1;
  if (!state_valid(node + cchange(Dir(0, -1, -1))))
    ret |= 1<<2;
  if (!state_valid(node + cchange(Dir(-1, 0, 0))))
    ret |= 1<<3;
  if (!state_valid(node + cchange(Dir(0, -1, 0))))
    ret |= 1<<4;
  if (!state_valid(node + cchange(Dir(-1, 0, 1))))
    ret |= 1<<5;
  if (!state_valid(node + cchange(Dir(0, 0, 1))))
    ret |= 1<<6;
  if (!state_valid(node + cchange(Dir(0, -1, 1))))
    ret |= 1<<7;
 return ret;
}

uint8_t JPS::encode_obs_3d(const Point & node, const Point & parent, const std::function<bool(const Point &)> & state_valid) {
  Dir (*cchange)(const Dir & dir) = standardize_dir(node.direction_to(parent));
  
  uint8_t ret = 0;
  if (!state_valid(node + cchange(Dir(-1, 0, -1))))
    ret |= 1;
  if (!state_valid(node + cchange(Dir(0, 0, -1))))
    ret |= 1<<1;
  if (!state_valid(node + cchange(Dir(0, -1, -1))))
    ret |= 1<<2;
  if (!state_valid(node + cchange(Dir(-1, 0, 0))))
    ret |= 1<<3;
  if (!state_valid(node + cchange(Dir(-1, -1, 0))))
    ret |= 1<<4;   
  if (!state_valid(node + cchange(Dir(0, -1, 0))))
    ret |= 1<<5;
  return ret;
}

std::set<Dir> JPS::decode_obs_1d(const uint8_t & id){
  std::set<Dir> ret;
  if (id & 1)
    ret.insert(Dir(0,-1,-1));
  if (id & (1<<1))
    ret.insert(Dir(0,0,-1));
  if (id & (1<<2))
    ret.insert(Dir(0, 1,-1));
  if (id & (1<<3))
    ret.insert(Dir(0,-1,0));
  if (id & (1<<4))
    ret.insert(Dir(0,1,0));
  if (id & (1<<5))
    ret.insert(Dir(0,-1,1));
  if (id & (1<<6))
    ret.insert(Dir(0,0,1));
  if (id & (1<<7))
    ret.insert(Dir(0,1,1));
  return move(ret);
}

std::set<Dir> JPS::decode_obs_2d(const uint8_t & id){
  std::set<Dir> ret;
  if (id & 1)
    ret.insert(Dir(-1,0,-1));
  if (id & (1<<1))
    ret.insert(Dir(0,0,-1));
  if (id & (1<<2))
    ret.insert(Dir(0, -1,-1));
  if (id & (1<<3))
    ret.insert(Dir(-1,0,0));
  if (id & (1<<4))
    ret.insert(Dir(0,-1,0));
  if (id & (1<<5))
    ret.insert(Dir(-1,0,1));
  if (id & (1<<6))
    ret.insert(Dir(0,0,1));
  if (id & (1<<7))
    ret.insert(Dir(0,-1,1));
  return move(ret);
}

std::set<Dir> JPS::decode_obs_3d(const uint8_t & id){
  std::set<Dir> ret;
  if (id & 1)
    ret.insert(Dir(-1,0,-1));
  if (id & (1<<1))
    ret.insert(Dir(0,0,-1));
  if (id & (1<<2))
    ret.insert(Dir(0, -1,-1));
  if (id & (1<<3))
    ret.insert(Dir(-1,0,0));
  if (id & (1<<4))
    ret.insert(Dir(-1,-1,0));
  if (id & (1<<5))
    ret.insert(Dir(0,-1,0));
  return move(ret);
}

uint8_t JPS::encode_fn_1d(const std::set<Dir> & fn_set){
  uint8_t id = 0;
  if (fn_set.find(Dir(1,-1,-1)) != fn_set.end())
    id |= 1;
  if (fn_set.find(Dir(1,0,-1)) != fn_set.end())
    id |= 1<<1;
  if (fn_set.find(Dir(1,1,-1)) != fn_set.end())
    id |= 1<<2;
  if (fn_set.find(Dir(1,-1,0)) != fn_set.end())
    id |= 1<<3;
  if (fn_set.find(Dir(1,1,0)) != fn_set.end())
    id |= 1<<4;
  if (fn_set.find(Dir(1,-1,1)) != fn_set.end())
    id |= 1<<5;
  if (fn_set.find(Dir(1,0,1)) != fn_set.end())
    id |= 1<<6;
  if (fn_set.find(Dir(1,1,1)) != fn_set.end())
    id |= 1<<7;
  return id;
}

std::set<Dir> JPS::decode_fn_1d(const Point & node, const Point & parent, const uint8_t & id){
  Dir (*cchange)(const Dir & dir) = standardize_dir(node.direction_to(parent));

  std::set<Dir> ret;
  if (id & 1)
    ret.insert(cchange(Dir(1,-1,-1)));
  if (id & (1<<1))
    ret.insert(cchange(Dir(1,0,-1)));
  if (id & (1<<2))
    ret.insert(cchange(Dir(1,1,-1)));
  if (id & (1<<3))
    ret.insert(cchange(Dir(1,-1,0)));
  if (id & (1<<4))
    ret.insert(cchange(Dir(1,1,0)));
  if (id & (1<<5))
    ret.insert(cchange(Dir(1,-1,1)));
  if (id & (1<<6))
    ret.insert(cchange(Dir(1,0,1)));
  if (id & (1<<7))
    ret.insert(cchange(Dir(1,1,1)));
  return move(ret);
}

uint16_t JPS::encode_fn_2d(const std::set<Dir> & fn_set){
  uint16_t id = 0;
  if (fn_set.find(Dir(-1,1,-1)) != fn_set.end())
    id |= 1;
  if (fn_set.find(Dir(0,1,-1)) != fn_set.end())
    id |= 1<<1;
  if (fn_set.find(Dir(1,1,-1)) != fn_set.end())
    id |= 1<<2; 
  if (fn_set.find(Dir(1,0,-1)) != fn_set.end())
    id |= 1<<3;
  if (fn_set.find(Dir(1,-1,-1)) != fn_set.end())
    id |= 1<<4;

  if (fn_set.find(Dir(-1,1,0)) != fn_set.end())
    id |= 1<<5;
  if (fn_set.find(Dir(1,-1,0)) != fn_set.end())
    id |= 1<<6;

  if (fn_set.find(Dir(-1,1,1)) != fn_set.end())
    id |= 1<<7;
  if (fn_set.find(Dir(0,1,1)) != fn_set.end())
    id |= 1<<8;
  if (fn_set.find(Dir(1,1,1)) != fn_set.end())
    id |= 1<<9;
  if (fn_set.find(Dir(1,0,1)) != fn_set.end())
    id |= 1<<10;
  if (fn_set.find(Dir(1,-1,1)) != fn_set.end())
    id |= 1<<11;

  return id;
}

std::set<Dir> JPS::decode_fn_2d(const Point & node, const Point & parent, const uint16_t & id){
  Dir (*cchange)(const Dir & dir) = standardize_dir(node.direction_to(parent));

  std::set<Dir> ret;
  if (id & 1)
    ret.insert(cchange(Dir(-1,1,-1)));
  if (id & (1<<1))
    ret.insert(cchange(Dir(0,1,-1)));
  if (id & (1<<2))
    ret.insert(cchange(Dir(1,1,-1)));
  if (id & (1<<3))
    ret.insert(cchange(Dir(1,0,-1)));
  if (id & (1<<4))
    ret.insert(cchange(Dir(1,-1,-1)));
  if (id & (1<<5))
    ret.insert(cchange(Dir(-1,1,0)));
  if (id & (1<<6))
    ret.insert(cchange(Dir(1,-1,0)));
  if (id & (1<<7))
    ret.insert(cchange(Dir(-1,1,1)));
  if (id & (1<<8))
    ret.insert(cchange(Dir(0,1,1)));
  if (id & (1<<9))
    ret.insert(cchange(Dir(1,1,1)));
  if (id & (1<<10))
    ret.insert(cchange(Dir(1,0,1)));
  if (id & (1<<11))
    ret.insert(cchange(Dir(1,-1,1)));  
  return move(ret);
}

uint16_t JPS::encode_fn_3d(const std::set<Dir> & fn_set){
  uint16_t id = 0;
  if (fn_set.find(Dir(-1,1,-1)) != fn_set.end())
    id |= 1;
  if (fn_set.find(Dir(0,1,-1)) != fn_set.end())
    id |= 1<<1;
  if (fn_set.find(Dir(1,1,-1)) != fn_set.end())
    id |= 1<<2; 
  if (fn_set.find(Dir(1,0,-1)) != fn_set.end())
    id |= 1<<3;
  if (fn_set.find(Dir(1,-1,-1)) != fn_set.end())
    id |= 1<<4;

  if (fn_set.find(Dir(-1,1,0)) != fn_set.end())
    id |= 1<<5;
  if (fn_set.find(Dir(1,-1,0)) != fn_set.end())
    id |= 1<<6;

  if (fn_set.find(Dir(-1,1,1)) != fn_set.end())
    id |= 1<<7;
  if (fn_set.find(Dir(-1,0,1)) != fn_set.end())
    id |= 1<<8;
  if (fn_set.find(Dir(-1,-1,1)) != fn_set.end())
    id |= 1<<9;
  if (fn_set.find(Dir(0,-1,1)) != fn_set.end())
    id |= 1<<10;
  if (fn_set.find(Dir(1,-1,1)) != fn_set.end())
    id |= 1<<11;

  return id;
}

std::set<Dir> JPS::decode_fn_3d(const Point & node, const Point & parent, const uint16_t & id){
  Dir (*cchange)(const Dir & dir) = standardize_dir(node.direction_to(parent));

  std::set<Dir> ret;
  if (id & 1)
    ret.insert(cchange(Dir(-1,1,-1)));
  if (id & (1<<1))
    ret.insert(cchange(Dir(0,1,-1)));
  if (id & (1<<2))
    ret.insert(cchange(Dir(1,1,-1)));
  if (id & (1<<3))
    ret.insert(cchange(Dir(1,0,-1)));
  if (id & (1<<4))
    ret.insert(cchange(Dir(1,-1,-1)));
  if (id & (1<<5))
    ret.insert(cchange(Dir(-1,1,0)));
  if (id & (1<<6))
    ret.insert(cchange(Dir(1,-1,0)));
  if (id & (1<<7))
    ret.insert(cchange(Dir(-1,1,1)));
  if (id & (1<<8))
    ret.insert(cchange(Dir(-1,0,1)));
  if (id & (1<<9))
    ret.insert(cchange(Dir(-1,-1,1)));
  if (id & (1<<10))
    ret.insert(cchange(Dir(0,-1,1)));
  if (id & (1<<11))
    ret.insert(cchange(Dir(1,-1,1)));  
  return move(ret);
}
