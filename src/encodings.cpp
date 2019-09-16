#include "jps/jps.hpp"

using namespace std;
using namespace JPS;

function<Dir(const Dir &)> JPS::standardize_dir(const Dir & d0) {
  // coordinate change so that d0 becomes one of [-1 0 0] [-1 -1 0] or [-1 -1 -1]
  // and s.t. coordinate change is its own inverse

  if (d0.order() == 1) {
    if (d0.dx() == -1)  // identity 
      return [] (const Dir & dir) {return dir;};
    if (d0.dx() == 1)  // mirror
      return [] (const Dir & dir) {return -dir;};
    if (d0.dy() == -1)  // permute x,y
      return [] (const Dir & dir) {return Dir(dir.dy(), dir.dx(), dir.dz());};
    if (d0.dy() == 1)  // permute x,y and mirror
      return [] (const Dir & dir) {return Dir(-dir.dy(), -dir.dx(), dir.dz());};
    if (d0.dz() == -1)  // permute x,z
      return [] (const Dir & dir) {return Dir(dir.dz(), dir.dy(), dir.dx());};
    if (d0.dz() == 1)  // permute x,z and mirror
      return [] (const Dir & dir) {return Dir(-dir.dz(), dir.dy(), -dir.dx());};
  }

  if (d0.order() == 2) {
    if (d0.dx() == -1 && d0.dy() == -1)  // identity 
      return [] (const Dir & dir) {return dir;};
    if (d0.dx() == -1 && d0.dy() == 1)  // flip y
      return [] (const Dir & dir) {return Dir(dir.dx(), -dir.dy(), dir.dz());};
    if (d0.dx() == 1 && d0.dy() == 1)  // flip x,y 
      return [] (const Dir & dir) {return -dir;};
    if (d0.dx() == 1 && d0.dy() == -1)  // flip x
      return [] (const Dir & dir) {return Dir(-dir.dx(), dir.dy(), dir.dz());};

    if (d0.dx() == -1 && d0.dz() == -1)  // flip y/z 
      return [] (const Dir & dir) {return Dir(dir.dx(), dir.dz(), dir.dy());};
    if (d0.dx() == -1 && d0.dz() == 1)   // change-flip y/z
      return [] (const Dir & dir) {return Dir(dir.dx(), -dir.dz(), -dir.dy());};
    if (d0.dx() == 1 && d0.dz() == 1)  // flip x, change-flip y/z
      return [] (const Dir & dir) {return Dir(-dir.dx(), -dir.dz(), -dir.dy());};
    if (d0.dx() == 1 && d0.dz() == -1)  // flip x, change y/z
      return [] (const Dir & dir) {return Dir(-dir.dx(), dir.dz(), dir.dy());};

    if (d0.dy() == -1 && d0.dz() == -1)  // flip x/z 
      return [] (const Dir & dir) {return Dir(dir.dz(), dir.dy(), dir.dx());};
    if (d0.dy() == -1 && d0.dz() == 1)  // change-flip x/z 
      return [] (const Dir & dir) {return Dir(-dir.dz(), dir.dy(), -dir.dx());};
    if (d0.dy() == 1 && d0.dz() == 1)  // flip y, change-flip x/z
      return [] (const Dir & dir) {return Dir(-dir.dz(), -dir.dy(), -dir.dx());};
    if (d0.dy() == 1 && d0.dz() == -1)  // flip y, change x/z
      return [] (const Dir & dir) {return Dir(dir.dz(), -dir.dy(), dir.dx());};
  }

  if (d0.order() == 3) {
    if (d0.dx() == -1 && d0.dy() == -1 && d0.dz() == -1)  // identity 
      return [] (const Dir & dir) {return dir;};
    if (d0.dx() == 1 && d0.dy() == 1 && d0.dz() == 1)  // flip all 
      return [] (const Dir & dir) {return -dir;};

    if (d0.dx() == 1 && d0.dy() == 1 && d0.dz() == -1)  // flip x,y 
      return [] (const Dir & dir) {return Dir(-dir.dx(), -dir.dy(), dir.dz());};
    if (d0.dx() == 1 && d0.dy() == -1 && d0.dz() == 1)  // flip x,z 
      return [] (const Dir & dir) {return Dir(-dir.dx(), dir.dy(), -dir.dz());};
    if (d0.dx() == -1 && d0.dy() == 1 && d0.dz() == 1)  // flip y,z
      return [] (const Dir & dir) {return Dir(dir.dx(), -dir.dy(), -dir.dz());};

    if (d0.dx() == -1 && d0.dy() == -1 && d0.dz() == 1)  // flip z
      return [] (const Dir & dir) {return Dir(dir.dx(), dir.dy(), -dir.dz());};
    if (d0.dx() == -1 && d0.dy() == 1 && d0.dz() == -1)  // flip y
      return [] (const Dir & dir) {return Dir(dir.dx(), -dir.dy(), dir.dz());};
    if (d0.dx() == 1 && d0.dy() == -1 && d0.dz() == -1)  // flip x
      return [] (const Dir & dir) {return Dir(-dir.dx(), dir.dy(), dir.dz());};
  }

  return [] (const Dir & dir) {return Dir(0,0,0);};
}

uint8_t JPS::encode_obs_1d(std::function<bool(const Dir &)> is_valid){
	uint8_t ret = 0;
  if (!is_valid(Dir(0, -1, -1)))
    ret |= 1;
  if (!is_valid(Dir(0, 0, -1)))
    ret |= 1<<1;
  if (!is_valid(Dir(0, 1, -1)))
    ret |= 1<<2;
  if (!is_valid(Dir(0, -1, 0)))
    ret |= 1<<3;
  if (!is_valid(Dir(0, 1, 0)))
    ret |= 1<<4;
  if (!is_valid(Dir(0, -1, 1)))
    ret |= 1<<5;
  if (!is_valid(Dir(0, 0, 1)))
    ret |= 1<<6;
  if (!is_valid(Dir(0, 1, 1)))
    ret |= 1<<7;
  return ret;
}

std::set<Dir> JPS::decode_obs_1d(uint8_t id){
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

uint8_t JPS::encode_obs_2d(std::function<bool(const Dir &)> is_valid){
	uint8_t ret = 0;
  if (!is_valid(Dir(-1, 0, -1)))
    ret |= 1;
  if (!is_valid(Dir(0, 0, -1)))
    ret |= 1<<1;
  if (!is_valid(Dir(0, -1, -1)))
    ret |= 1<<2;
  if (!is_valid(Dir(-1, 0, 0)))
    ret |= 1<<3;
  if (!is_valid(Dir(0, -1, 0)))
    ret |= 1<<4;
  if (!is_valid(Dir(-1, 0, 1)))
    ret |= 1<<5;
  if (!is_valid(Dir(0, 0, 1)))
    ret |= 1<<6;
  if (!is_valid(Dir(0, -1, 1)))
    ret |= 1<<7;
 return ret;
}

std::set<Dir> JPS::decode_obs_2d(uint8_t id){
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

uint8_t JPS::encode_obs_3d(std::function<bool(const Dir &)> is_valid){
	uint8_t ret = 0;
  if (!is_valid(Dir(-1, 0, -1)))
    ret |= 1;
  if (!is_valid(Dir(0, 0, -1)))
    ret |= 1<<1;
  if (!is_valid(Dir(0, -1, -1)))
    ret |= 1<<2;
  if (!is_valid(Dir(-1, 0, 0)))
    ret |= 1<<3;
  if (!is_valid(Dir(-1, -1, 0)))
    ret |= 1<<4;   
  if (!is_valid(Dir(0, -1, 0)))
    ret |= 1<<5;
	return ret;
}

std::set<Dir> JPS::decode_obs_3d(uint8_t id){
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

uint8_t JPS::encode_fn_1d(std::function<bool(const Dir &)> is_forced){
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

std::set<Dir> JPS::decode_fn_1d(uint8_t id){
	std::set<Dir> ret;
  if (id & 1)
    ret.insert(Dir(1,-1,-1));
  if (id & (1<<1))
    ret.insert(Dir(1,0,-1));
  if (id & (1<<2))
    ret.insert(Dir(1,1,-1));
  if (id & (1<<3))
    ret.insert(Dir(1,-1,0));
  if (id & (1<<4))
    ret.insert(Dir(1,1,0));
  if (id & (1<<5))
    ret.insert(Dir(1,-1,1));
  if (id & (1<<6))
    ret.insert(Dir(1,0,1));
  if (id & (1<<7))
    ret.insert(Dir(1,1,1));
  return move(ret);
}

uint16_t JPS::encode_fn_2d(std::function<bool(const Dir &)> is_forced){
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

std::set<Dir> JPS::decode_fn_2d(uint16_t id){
	std::set<Dir> ret;
  if (id & 1)
    ret.insert(Dir(-1,1,-1));
  if (id & (1<<1))
    ret.insert(Dir(0,1,-1));
  if (id & (1<<2))
    ret.insert(Dir(1,1,-1));
  if (id & (1<<3))
    ret.insert(Dir(1,0,-1));
  if (id & (1<<4))
    ret.insert(Dir(1,-1,-1));
  if (id & (1<<5))
    ret.insert(Dir(-1,1,0));
  if (id & (1<<6))
    ret.insert(Dir(1,-1,0));
  if (id & (1<<7))
    ret.insert(Dir(-1,1,1));
  if (id & (1<<8))
    ret.insert(Dir(0,1,1));
  if (id & (1<<9))
    ret.insert(Dir(1,1,1));
  if (id & (1<<10))
    ret.insert(Dir(1,0,1));
  if (id & (1<<11))
    ret.insert(Dir(1,-1,1));  
  return move(ret);
}

uint16_t JPS::encode_fn_3d(std::function<bool(const Dir &)> is_forced){
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

std::set<Dir> JPS::decode_fn_3d(uint16_t id){
	std::set<Dir> ret;
  if (id & 1)
    ret.insert(Dir(-1,1,-1));
  if (id & (1<<1))
    ret.insert(Dir(0,1,-1));
  if (id & (1<<2))
    ret.insert(Dir(1,1,-1));
  if (id & (1<<3))
    ret.insert(Dir(1,0,-1));
  if (id & (1<<4))
    ret.insert(Dir(1,-1,-1));
  if (id & (1<<5))
    ret.insert(Dir(-1,1,0));
  if (id & (1<<6))
    ret.insert(Dir(1,-1,0));
  if (id & (1<<7))
    ret.insert(Dir(-1,1,1));
  if (id & (1<<8))
    ret.insert(Dir(-1,0,1));
  if (id & (1<<9))
    ret.insert(Dir(-1,-1,1));
  if (id & (1<<10))
    ret.insert(Dir(0,-1,1));
  if (id & (1<<11))
    ret.insert(Dir(1,-1,1));  
  return move(ret);
}
