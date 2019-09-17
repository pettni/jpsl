#include <iostream>
#include <set>
#include "gtest/gtest.h"

#include "jpsl/jpsl.hpp"
#include "jpsl/encodings.hpp"

using namespace std;
using namespace JPSL;

TEST(test_encodings, test_encodings) {

  for (uint16_t prob=0; prob != (1<<7); ++prob) {
    set<Dir> dec = decode_obs_1d(prob);
    uint16_t enc = encode_obs_1d(Point(0,0,0), Point(-1,0,0), [dec] (const Point & p) {
      return dec.find(Point(0,0,0).direction_to(p)) == dec.end();
    });
    EXPECT_EQ(prob, enc);
  }

  for (uint16_t prob=0; prob != (1<<7); ++prob) {
    set<Dir> dec = decode_obs_2d(prob);
    uint16_t enc = encode_obs_2d(Point(0,0,0), Point(-1,-1,0), [dec] (const Point & p) {
      return dec.find(Point(0,0,0).direction_to(p)) == dec.end();
    });
    EXPECT_EQ(prob, enc);
  }

  for (uint16_t prob=0; prob != (1<<5); ++prob) {
    set<Dir> dec = decode_obs_3d(prob);
    uint16_t enc = encode_obs_3d(Point(0,0,0), Point(-1,-1,-1), [dec] (const Point & p) {
      return dec.find(Point(0,0,0).direction_to(p)) == dec.end();
    });
    EXPECT_EQ(prob, enc);
  }

  for (uint16_t prob=0; prob != (1<<7); ++prob) {
    set<Dir> dec = decode_fn_1d(Point(0,0,0), Point(-1,0,0), prob);
    EXPECT_EQ(prob, encode_fn_1d(dec));
  }

  for (uint16_t prob=0; prob != (1<<11); ++prob) {
    set<Dir> dec = decode_fn_2d(Point(0,0,0), Point(-1,0,0), prob);
    EXPECT_EQ(prob, encode_fn_2d(dec));
  }

  for (uint16_t prob=0; prob != (1<<5); ++prob) {
    set<Dir> dec = decode_fn_3d(Point(0,0,0), Point(-1,0,0), prob);
    EXPECT_EQ(prob, encode_fn_3d(dec));
  }

}