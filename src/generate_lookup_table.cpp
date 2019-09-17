#include <iostream>
#include <fstream>

#include "jpsl/jpsl.hpp"
#include "jpsl/encodings.hpp"

using namespace std;
using namespace JPSL;

int main(int argc, char const *argv[]) {

  ofstream datafile;
  datafile.open ("lookup_table.cpp");

  // 1D problems
  datafile << "uint8_t const JPS::lookup1d[256] = {";
  for (uint16_t iter=0; iter!=256; ++iter) {
    set<Dir> obstacles = decode_obs_1d(uint8_t(iter));
    set<Dir> forced = forced_neighbors_slow(Point(0,0,0), Point(-1,0,0), [obstacles] (const Point & p) {
      return obstacles.find(Point(0,0,0).direction_to(p)) == obstacles.end();});
    uint8_t neighbors = encode_fn_1d(forced);
    datafile << (int) neighbors;
    if (iter < 255)
      datafile << ", ";
  }
  datafile << "};" << endl;
  // 2D problems
  datafile << "uint16_t const JPS::lookup2d[256] = {";  
  for (uint16_t iter=0; iter!=256; ++iter) {
    set<Dir> obstacles = decode_obs_2d(uint8_t(iter));
    set<Dir> forced = forced_neighbors_slow(Point(0,0,0), Point(-1,-1,0), [obstacles] (const Point & p) {
      return obstacles.find(Point(0,0,0).direction_to(p)) == obstacles.end();});
    uint16_t neighbors = encode_fn_2d(forced);
    datafile << (int) neighbors;
    if (iter < 255)
      datafile << ", ";
  }
  datafile << "};" << endl;  
  // 3D problems
  datafile << "uint16_t const JPS::lookup3d[64] = {";    
  for (uint16_t iter=0; iter!=64; ++iter) {
    set<Dir> obstacles = decode_obs_3d(uint8_t(iter));
    set<Dir> forced = forced_neighbors_slow(Point(0,0,0), Point(-1,-1,-1), [obstacles] (const Point & p) {
      return obstacles.find(Point(0,0,0).direction_to(p)) == obstacles.end();});
    uint16_t neighbors = encode_fn_3d(forced);
    datafile << (int) neighbors;
    if (iter < 63)
      datafile << ", ";
  }
  datafile << "};" << endl;  

  datafile.close();

  return 0;
}