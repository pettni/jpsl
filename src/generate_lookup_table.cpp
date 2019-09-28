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
  datafile << "uint8_t const JPSL::lookup1d[256] = {";
  for (uint16_t iter=0; iter!=256; ++iter) {
    vector<Dir> obstacles = decode_obs<1>(uint8_t(iter));
    vector<Dir> forced = forced_neighbors_slow(Point(0,0,0), Point(-1,0,0), [obstacles] (const Point & p) {
      return find(obstacles.begin(), obstacles.end(), Point(0,0,0).direction_to(p)) == obstacles.end();});
    uint8_t neighbors = encode_fn<1>(forced);
    datafile << (int) neighbors;
    if (iter < 255)
      datafile << ", ";
  }
  datafile << "};" << endl;
  // 2D problems
  datafile << "uint16_t const JPSL::lookup2d[256] = {";  
  for (uint16_t iter=0; iter!=256; ++iter) {
    vector<Dir> obstacles = decode_obs<2>(uint8_t(iter));
    vector<Dir> forced = forced_neighbors_slow(Point(0,0,0), Point(-1,-1,0), [obstacles] (const Point & p) {
      return find(obstacles.begin(), obstacles.end(), Point(0,0,0).direction_to(p)) == obstacles.end();});
    uint16_t neighbors = encode_fn<2>(forced);
    datafile << (int) neighbors;
    if (iter < 255)
      datafile << ", ";
  }
  datafile << "};" << endl;  
  // 3D problems
  datafile << "uint16_t const JPSL::lookup3d[64] = {";    
  for (uint16_t iter=0; iter!=64; ++iter) {
    vector<Dir> obstacles = decode_obs<3>(uint8_t(iter));
    vector<Dir> forced = forced_neighbors_slow(Point(0,0,0), Point(-1,-1,-1), [obstacles] (const Point & p) {
      return find(obstacles.begin(), obstacles.end(), Point(0,0,0).direction_to(p)) == obstacles.end();});
    uint16_t neighbors = encode_fn<3>(forced);
    datafile << (int) neighbors;
    if (iter < 63)
      datafile << ", ";
  }
  datafile << "};" << endl;  

  datafile.close();

  return 0;
}