#include <iostream>
#include <set>
#include "gtest/gtest.h"

#include "jps/jps.hpp"

using namespace std;
using namespace JPS;

TEST(TestSuite, test_encodings) {

	for (uint16_t prob=0; prob != (1<<7); ++prob) {
		set<Dir> dec = decode_obs_1d(prob);
		uint16_t enc = encode_obs_1d([dec] (const Dir & d) {return dec.find(d) == dec.end();});
		EXPECT_EQ(prob, enc);
	}

	for (uint16_t prob=0; prob != (1<<7); ++prob) {
		set<Dir> dec = decode_obs_2d(prob);
		uint16_t enc = encode_obs_2d([dec] (const Dir & d) {return dec.find(d) == dec.end();});
		EXPECT_EQ(prob, enc);
	}

	for (uint16_t prob=0; prob != (1<<5); ++prob) {
		set<Dir> dec = decode_obs_3d(prob);
		uint16_t enc = encode_obs_3d([dec] (const Dir & d) {return dec.find(d) == dec.end();});
		EXPECT_EQ(prob, enc);
	}

	for (uint16_t prob=0; prob != (1<<7); ++prob) {
		set<Dir> dec = decode_fn_1d(prob);
		EXPECT_EQ(prob, encode_fn_1d([dec] (const Dir & d) {return dec.find(d) != dec.end();}));
	}

	for (uint16_t prob=0; prob != (1<<11); ++prob) {
		set<Dir> dec = decode_fn_2d(prob);
		EXPECT_EQ(prob, encode_fn_2d([dec] (const Dir & d) {return dec.find(d) != dec.end();}));
	}

	for (uint16_t prob=0; prob != (1<<5); ++prob) {
		set<Dir> dec = decode_fn_3d(prob);
		EXPECT_EQ(prob, encode_fn_3d([dec] (const Dir & d) {return dec.find(d) != dec.end();}));
	}

}