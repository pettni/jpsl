#include "jps/jps.hpp"

using namespace std;
using namespace JPS;

// 1D coordinate changes
Dir JPS::fnd_01(const Dir & dir) {
  return dir;
};

Dir JPS::fnd_02(const Dir & dir) {
  return -dir;
};

Dir JPS::fnd_03(const Dir & dir) {
  return Dir(dir.dy(), dir.dx(), dir.dz());
};

Dir JPS::fnd_04(const Dir & dir) {
  return Dir(-dir.dy(), -dir.dx(), dir.dz());
};

Dir JPS::fnd_05(const Dir & dir) {
  return Dir(dir.dz(), dir.dy(), dir.dx());
};

Dir JPS::fnd_06(const Dir & dir) {
  return Dir(-dir.dz(), dir.dy(), -dir.dx());
};

// 2D coordinate changes
Dir JPS::fnd_07(const Dir & dir) {
  return dir;
};

Dir JPS::fnd_08(const Dir & dir) {
  return Dir(dir.dx(), -dir.dy(), dir.dz());;
};

Dir JPS::fnd_09(const Dir & dir) {
  return -dir;;
};

Dir JPS::fnd_10(const Dir & dir) {
  return Dir(-dir.dx(), dir.dy(), dir.dz());;
};

Dir JPS::fnd_11(const Dir & dir) {
  return Dir(dir.dx(), dir.dz(), dir.dy());;
};

Dir JPS::fnd_12(const Dir & dir) {
  return Dir(dir.dx(), -dir.dz(), -dir.dy());;
};

Dir JPS::fnd_13(const Dir & dir) {
  return Dir(-dir.dx(), -dir.dz(), -dir.dy());;
};

Dir JPS::fnd_14(const Dir & dir) {
  return Dir(-dir.dx(), dir.dz(), dir.dy());;
};

Dir JPS::fnd_15(const Dir & dir) {
  return Dir(dir.dz(), dir.dy(), dir.dx());;
};

Dir JPS::fnd_16(const Dir & dir) {
  return Dir(-dir.dz(), dir.dy(), -dir.dx());;
};

Dir JPS::fnd_17(const Dir & dir) {
  return Dir(-dir.dz(), -dir.dy(), -dir.dx());;
};

Dir JPS::fnd_18(const Dir & dir) {
  return Dir(dir.dz(), -dir.dy(), dir.dx());;
};


// 3D coordinate changes
Dir JPS::fnd_19(const Dir & dir) {
  return dir;
};

Dir JPS::fnd_20(const Dir & dir) {
  return -dir;
};

Dir JPS::fnd_21(const Dir & dir) {
  return Dir(-dir.dx(), -dir.dy(), dir.dz());
};

Dir JPS::fnd_22(const Dir & dir) {
  return Dir(-dir.dx(), dir.dy(), -dir.dz());
};

Dir JPS::fnd_23(const Dir & dir) {
  return Dir(dir.dx(), -dir.dy(), -dir.dz());
};

Dir JPS::fnd_24(const Dir & dir) {
  return Dir(dir.dx(), dir.dy(), -dir.dz());
};

Dir JPS::fnd_25(const Dir & dir) {
  return Dir(dir.dx(), -dir.dy(), dir.dz());
};

Dir JPS::fnd_26(const Dir & dir) {
  return Dir(-dir.dx(), dir.dy(), dir.dz());
};


Dir(*JPS::standardize_dir(const Dir & d0))(const Dir &) {
  // coordinate change so that d0 becomes one of [-1 0 0] [-1 -1 0] or [-1 -1 -1]
  // and s.t. coordinate change is its own inverse

  if (d0.order() == 1) {
    if (d0.dx() == -1)  // identity 
      return &fnd_01;
    if (d0.dx() == 1)  // mirror
      return &fnd_02;
    if (d0.dy() == -1)  // permute x,y
      return &fnd_03;
    if (d0.dy() == 1)  // permute x,y and mirror
      return &fnd_04;
    if (d0.dz() == -1)  // permute x,z
      return &fnd_05;
    if (d0.dz() == 1)  // permute x,z and mirror
      return &fnd_06;
  }

  if (d0.order() == 2) {
    if (d0.dx() == -1 && d0.dy() == -1)  // identity 
      return &fnd_07;
    if (d0.dx() == -1 && d0.dy() == 1)  // flip y
      return &fnd_08;
    if (d0.dx() == 1 && d0.dy() == 1)  // flip x,y 
      return &fnd_09;
    if (d0.dx() == 1 && d0.dy() == -1)  // flip x
      return &fnd_10;

    if (d0.dx() == -1 && d0.dz() == -1)  // flip y/z 
      return &fnd_11;
    if (d0.dx() == -1 && d0.dz() == 1)   // change-flip y/z
      return &fnd_12;
    if (d0.dx() == 1 && d0.dz() == 1)  // flip x, change-flip y/z
      return &fnd_13;
    if (d0.dx() == 1 && d0.dz() == -1)  // flip x, change y/z
      return &fnd_14;

    if (d0.dy() == -1 && d0.dz() == -1)  // flip x/z 
      return &fnd_15;
    if (d0.dy() == -1 && d0.dz() == 1)  // change-flip x/z 
      return &fnd_16;
    if (d0.dy() == 1 && d0.dz() == 1)  // flip y, change-flip x/z
      return &fnd_17;
    if (d0.dy() == 1 && d0.dz() == -1)  // flip y, change x/z
      return &fnd_18;
  }

  if (d0.order() == 3) {
    if (d0.dx() == -1 && d0.dy() == -1 && d0.dz() == -1)  // identity 
      return &fnd_19;
    if (d0.dx() == 1 && d0.dy() == 1 && d0.dz() == 1)  // flip all 
      return &fnd_20;

    if (d0.dx() == 1 && d0.dy() == 1 && d0.dz() == -1)  // flip x,y 
      return &fnd_21;
    if (d0.dx() == 1 && d0.dy() == -1 && d0.dz() == 1)  // flip x,z 
      return &fnd_22;
    if (d0.dx() == -1 && d0.dy() == 1 && d0.dz() == 1)  // flip y,z
      return &fnd_23;

    if (d0.dx() == -1 && d0.dy() == -1 && d0.dz() == 1)  // flip z
      return &fnd_24;
    if (d0.dx() == -1 && d0.dy() == 1 && d0.dz() == -1)  // flip y
      return &fnd_25;
    if (d0.dx() == 1 && d0.dy() == -1 && d0.dz() == -1)  // flip x
      return &fnd_26;
  }

  return &fnd_01;
}

uint8_t JPS::encode_obs_1d(const Point & node, const Point & parent, const std::function<bool(const Point &)> & is_valid) {

  Dir (*cchange)(const Dir & dir) = standardize_dir(node.incoming_dir(parent));

  uint8_t ret = 0;
  if (!is_valid(node + cchange(Dir(0, -1, -1))))
    ret |= 1;
  if (!is_valid(node + cchange(Dir(0, 0, -1))))
    ret |= 1<<1;
  if (!is_valid(node + cchange(Dir(0, 1, -1))))
    ret |= 1<<2;
  if (!is_valid(node + cchange(Dir(0, -1, 0))))
    ret |= 1<<3;
  if (!is_valid(node + cchange(Dir(0, 1, 0))))
    ret |= 1<<4;
  if (!is_valid(node + cchange(Dir(0, -1, 1))))
    ret |= 1<<5;
  if (!is_valid(node + cchange(Dir(0, 0, 1))))
    ret |= 1<<6;
  if (!is_valid(node + cchange(Dir(0, 1, 1))))
    ret |= 1<<7;
  return ret;
}

uint8_t JPS::encode_obs_2d(const Point & node, const Point & parent, const std::function<bool(const Point &)> & is_valid) {

  Dir (*cchange)(const Dir & dir) = standardize_dir(node.incoming_dir(parent));

  uint8_t ret = 0;
  if (!is_valid(node + cchange(Dir(-1, 0, -1))))
    ret |= 1;
  if (!is_valid(node + cchange(Dir(0, 0, -1))))
    ret |= 1<<1;
  if (!is_valid(node + cchange(Dir(0, -1, -1))))
    ret |= 1<<2;
  if (!is_valid(node + cchange(Dir(-1, 0, 0))))
    ret |= 1<<3;
  if (!is_valid(node + cchange(Dir(0, -1, 0))))
    ret |= 1<<4;
  if (!is_valid(node + cchange(Dir(-1, 0, 1))))
    ret |= 1<<5;
  if (!is_valid(node + cchange(Dir(0, 0, 1))))
    ret |= 1<<6;
  if (!is_valid(node + cchange(Dir(0, -1, 1))))
    ret |= 1<<7;
 return ret;
}

uint8_t JPS::encode_obs_3d(const Point & node, const Point & parent, const std::function<bool(const Point &)> & is_valid) {

  Dir (*cchange)(const Dir & dir) = standardize_dir(node.incoming_dir(parent));
  
  uint8_t ret = 0;
  if (!is_valid(node + cchange(Dir(-1, 0, -1))))
    ret |= 1;
  if (!is_valid(node + cchange(Dir(0, 0, -1))))
    ret |= 1<<1;
  if (!is_valid(node + cchange(Dir(0, -1, -1))))
    ret |= 1<<2;
  if (!is_valid(node + cchange(Dir(-1, 0, 0))))
    ret |= 1<<3;
  if (!is_valid(node + cchange(Dir(-1, -1, 0))))
    ret |= 1<<4;   
  if (!is_valid(node + cchange(Dir(0, -1, 0))))
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

uint8_t JPS::encode_fn_1d(const std::function<bool(const Dir &)> & is_forced){
	uint8_t id = 0;
  if (is_forced(Dir(1,-1,-1)))
    id |= 1;
  if (is_forced(Dir(1,0,-1)))
    id |= 1<<1;
  if (is_forced(Dir(1,1,-1)))
    id |= 1<<2;
  if (is_forced(Dir(1,-1,0)))
    id |= 1<<3;
  if (is_forced(Dir(1,1,0)))
    id |= 1<<4;
  if (is_forced(Dir(1,-1,1)))
    id |= 1<<5;
  if (is_forced(Dir(1,0,1)))
    id |= 1<<6;
  if (is_forced(Dir(1,1,1)))
    id |= 1<<7;
  return id;
}

std::set<Dir> JPS::decode_fn_1d(const Point & node, const Point & parent, const uint8_t & id){
  Dir (*cchange)(const Dir & dir) = standardize_dir(node.incoming_dir(parent));

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

uint16_t JPS::encode_fn_2d(const std::function<bool(const Dir &)> & is_forced){
	uint16_t id = 0;
  if (is_forced(Dir(-1,1,-1)))
    id |= 1;
  if (is_forced(Dir(0,1,-1)))
    id |= 1<<1;
  if (is_forced(Dir(1,1,-1)))
    id |= 1<<2; 
  if (is_forced(Dir(1,0,-1)))
    id |= 1<<3;
  if (is_forced(Dir(1,-1,-1)))
    id |= 1<<4;

  if (is_forced(Dir(-1,1,0)))
    id |= 1<<5;
  if (is_forced(Dir(1,-1,0)))
    id |= 1<<6;

  if (is_forced(Dir(-1,1,1)))
    id |= 1<<7;
  if (is_forced(Dir(0,1,1)))
    id |= 1<<8;
  if (is_forced(Dir(1,1,1)))
    id |= 1<<9;
  if (is_forced(Dir(1,0,1)))
    id |= 1<<10;
  if (is_forced(Dir(1,-1,1)))
    id |= 1<<11;

  return id;
}

std::set<Dir> JPS::decode_fn_2d(const Point & node, const Point & parent, const uint16_t & id){
  Dir (*cchange)(const Dir & dir) = standardize_dir(node.incoming_dir(parent));

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

uint16_t JPS::encode_fn_3d(const std::function<bool(const Dir &)> & is_forced){
	uint16_t id = 0;
  if (is_forced(Dir(-1,1,-1)))
    id |= 1;
  if (is_forced(Dir(0,1,-1)))
    id |= 1<<1;
  if (is_forced(Dir(1,1,-1)))
    id |= 1<<2; 
  if (is_forced(Dir(1,0,-1)))
    id |= 1<<3;
  if (is_forced(Dir(1,-1,-1)))
    id |= 1<<4;

  if (is_forced(Dir(-1,1,0)))
    id |= 1<<5;
  if (is_forced(Dir(1,-1,0)))
    id |= 1<<6;

  if (is_forced(Dir(-1,1,1)))
    id |= 1<<7;
  if (is_forced(Dir(-1,0,1)))
    id |= 1<<8;
  if (is_forced(Dir(-1,-1,1)))
    id |= 1<<9;
  if (is_forced(Dir(0,-1,1)))
    id |= 1<<10;
  if (is_forced(Dir(1,-1,1)))
    id |= 1<<11;

  return id;
}

std::set<Dir> JPS::decode_fn_3d(const Point & node, const Point & parent, const uint16_t & id){

  Dir (*cchange)(const Dir & dir) = standardize_dir(node.incoming_dir(parent));

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
