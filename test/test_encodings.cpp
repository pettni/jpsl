#include <iostream>
#include <set>
#include "gtest/gtest.h"

#include "jpsl/jpsl.hpp"
#include "jpsl/encodings.hpp"

using namespace std;
using namespace JPSL;

TEST(test_encodings, test_encodings) {

  for (uint16_t prob=0; prob != (1<<7); ++prob) {
    vector<Dir> dec = decode_obs<1>(prob);
    uint16_t enc = encode_obs<1>(Point(0,0,0), Point(-1,0,0), [dec] (const Point & p) {
      return find(dec.begin(), dec.end(), Point(0,0,0).direction_to(p)) == dec.end();
    });
    EXPECT_EQ(prob, enc);
  }

  for (uint16_t prob=0; prob != (1<<7); ++prob) {
    vector<Dir> dec = decode_obs<2>(prob);
    uint16_t enc = encode_obs<2>(Point(0,0,0), Point(-1,-1,0), [dec] (const Point & p) {
      return find(dec.begin(), dec.end(), Point(0,0,0).direction_to(p)) == dec.end();
    });
    EXPECT_EQ(prob, enc);
  }

  for (uint16_t prob=0; prob != (1<<5); ++prob) {
    vector<Dir> dec = decode_obs<3>(prob);
    uint16_t enc = encode_obs<3>(Point(0,0,0), Point(-1,-1,-1), [dec] (const Point & p) {
      return find(dec.begin(), dec.end(), Point(0,0,0).direction_to(p)) == dec.end();
    });
    EXPECT_EQ(prob, enc);
  }

  for (uint16_t prob=0; prob != (1<<7); ++prob) {
    vector<Dir> dec = decode_fn<1>(Point(0,0,0), Point(-1,0,0), prob);
    EXPECT_EQ(prob, encode_fn<1>(dec));
  }

  for (uint16_t prob=0; prob != (1<<11); ++prob) {
    vector<Dir> dec = decode_fn<2>(Point(0,0,0), Point(-1,0,0), prob);
    EXPECT_EQ(prob, encode_fn<2>(dec));
  }

  for (uint16_t prob=0; prob != (1<<5); ++prob) {
    vector<Dir> dec = decode_fn<3>(Point(0,0,0), Point(-1,0,0), prob);
    EXPECT_EQ(prob, encode_fn<3>(dec));
  }

}