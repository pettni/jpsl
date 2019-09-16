#ifndef JPS_ENC_HEADER 
#define JPS_ENC_HEADER

#include <functional>
#include <set>

#include "jps/point.hpp"
#include "jps/dir.hpp"

namespace JPS {

	typedef std::pair<JPS::Point, float> ASNode;

	const std::set<Dir> NEIGHBORS_3D({{-1,-1,-1}, {1,1,1}, \
																    {-1,-1,0}, {-1,0,-1}, {0,-1,-1}, \
																    {-1,0,0}, {0,-1,0}, {0,0,-1}, \
																    {-1,1,0}, {-1,0,1}, {0,-1,1}, \
																    {1,-1,0}, {1,0,-1}, {0,1,-1}, \
																    {-1,-1,1}, {-1,1,-1}, {1,-1,-1}, \
																    {-1,1,1}, {1,-1,1}, {1,1,-1}, \
																    {0,0,1}, {0,1,0}, {1,0,0}, \
																    {0,1,1}, {1,0,1}, {1,1,0}});

	extern uint8_t const lookup1d[256];
	extern uint16_t const lookup2d[256];
	extern uint16_t const lookup3d[64];

	Dir fnd_01(const Dir &);
	Dir fnd_02(const Dir &);
	Dir fnd_03(const Dir &);
	Dir fnd_04(const Dir &);
	Dir fnd_05(const Dir &);
	Dir fnd_06(const Dir &);
	Dir fnd_07(const Dir &);
	Dir fnd_08(const Dir &);
	Dir fnd_09(const Dir &);
	Dir fnd_10(const Dir &);
	Dir fnd_11(const Dir &);
	Dir fnd_12(const Dir &);
	Dir fnd_13(const Dir &);
	Dir fnd_14(const Dir &);
	Dir fnd_15(const Dir &);
	Dir fnd_16(const Dir &);
	Dir fnd_17(const Dir &);
	Dir fnd_18(const Dir &);
	Dir fnd_19(const Dir &);
	Dir fnd_20(const Dir &);
	Dir fnd_21(const Dir &);
	Dir fnd_22(const Dir &);
	Dir fnd_23(const Dir &);
	Dir fnd_24(const Dir &);
	Dir fnd_25(const Dir &);
	Dir fnd_26(const Dir &);

	// coordinate change to standard 3x3 problem
	Dir(*standardize_dir(const Dir &))(const Dir &);

	// encoding and decoding of 3x3 problems
	uint8_t encode_obs_1d(const Point &, const Point &, const std::function<bool(const Point &)> &);
	uint8_t encode_obs_2d(const Point &, const Point &, const std::function<bool(const Point &)> &);
	uint8_t encode_obs_3d(const Point &, const Point &, const std::function<bool(const Point &)> &);

	std::set<Dir> decode_obs_1d(const uint8_t &);
	std::set<Dir> decode_obs_2d(const uint8_t &);
	std::set<Dir> decode_obs_3d(const uint8_t &);

	uint8_t encode_fn_1d(const std::function<bool(const Dir &)> &);
	uint16_t encode_fn_2d(const std::function<bool(const Dir &)> &);
	uint16_t encode_fn_3d(const std::function<bool(const Dir &)> &);

	std::set<Dir> decode_fn_1d(const Point &, const Point &, const uint8_t &);
	std::set<Dir> decode_fn_2d(const Point &, const Point &, const uint16_t &);
	std::set<Dir> decode_fn_3d(const Point &, const Point &, const uint16_t &);

}

#endif
