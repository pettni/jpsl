#include <iostream>
#include <set>
#include "gtest/gtest.h"

#include "jps/jps.hpp"

using namespace std;
using namespace JPS;

TEST(TestSuite, test_encodings) {

	for (uint16_t prob=0; prob != (1<<7); ++prob)
		EXPECT_EQ(prob, encode_obs_1d(decode_obs_1d(prob)));

	for (uint16_t prob=0; prob != (1<<7); ++prob)
		EXPECT_EQ(prob, encode_obs_2d(decode_obs_2d(prob)));

	for (uint16_t prob=0; prob != (1<<5); ++prob)
		EXPECT_EQ(prob, encode_obs_3d(decode_obs_3d(prob)));

	for (uint16_t prob=0; prob != (1<<7); ++prob)
		EXPECT_EQ(prob, encode_fn_1d(decode_fn_1d(prob)));

	for (uint16_t prob=0; prob != (1<<11); ++prob)
		EXPECT_EQ(prob, encode_fn_2d(decode_fn_2d(prob)));

	for (uint16_t prob=0; prob != (1<<5); ++prob)
		EXPECT_EQ(prob, encode_fn_3d(decode_fn_3d(prob)));

}