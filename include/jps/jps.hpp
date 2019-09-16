#ifndef JPS_HEADER 
#define JPS_HEADER

#include <vector>
#include <queue>
#include <functional>

#include "jps/point.hpp"
#include "jps/dir.hpp"


namespace JPS {

	const std::set<Dir> NEIGHBORS_3D({{-1,-1,-1}, {1,1,1}, \
																    {-1,-1,0}, {-1,0,-1}, {0,-1,-1}, \
																    {-1,0,0}, {0,-1,0}, {0,0,-1}, \
																    {-1,1,0}, {-1,0,1}, {0,-1,1}, \
																    {1,-1,0}, {1,0,-1}, {0,1,-1}, \
																    {-1,-1,1}, {-1,1,-1}, {1,-1,-1}, \
																    {-1,1,1}, {1,-1,1}, {1,1,-1}, \
																    {0,0,1}, {0,1,0}, {1,0,0}, \
																    {0,1,1}, {1,0,1}, {1,1,0}});

	typedef std::pair<JPS::Point, float> ASNode;

	std::pair<std::vector<Point>, float> jps(Point, Point, std::function<bool(const Point &)>);

	std::pair<bool, Point> jump(const Point &, const Dir &, const Point &, std::function<bool(const Point &)>);

	std::set<Point> successors(const Point &, const Point &, const Point &, std::function<bool(const Point &)>);

	std::set<Dir> natural_neighbors(const Point &, const Point &, std::function<bool(const Point &)>);
	std::set<Dir> forced_neighbors(const Point &, const Point &, std::function<bool(const Point &)>);
	std::set<Dir> all_neighbors(const Point &, const Point &, std::function<bool(const Point &)>);

	std::set<Dir> natural_neighbors_(const Dir &, const std::set<Dir> &);
	std::set<Dir> forced_neighbors_(const Dir &, const std::set<Dir> &);
	std::set<Dir> all_neighbors_(const Dir &, const std::set<Dir> &);

	std::set<Dir> forced_neighbors_fast(const Dir &, std::function<bool(const Dir &)>);

	uint8_t get_obs_mask(const Dir &, std::function<bool(const Dir &)>);
	std::function<Dir(const Dir &)> standardize_dir(const Dir &);

	uint8_t encode_obs_1d(const std::set<Dir> &);
	uint8_t encode_obs_2d(const std::set<Dir> &);
	uint8_t encode_obs_3d(const std::set<Dir> &);

	std::set<Dir> decode_obs_1d(uint8_t);
	std::set<Dir> decode_obs_2d(uint8_t);
	std::set<Dir> decode_obs_3d(uint8_t);

	uint8_t encode_fn_1d(const std::set<Dir> &);
	uint16_t encode_fn_2d(const std::set<Dir> &);
	uint16_t encode_fn_3d(const std::set<Dir> &);

	std::set<Dir> decode_fn_1d(uint8_t);
	std::set<Dir> decode_fn_2d(uint16_t);
	std::set<Dir> decode_fn_3d(uint16_t);

	uint8_t forced_neighbors_fast_1d(uint8_t);
	uint8_t forced_neighbors_fast_2d(uint8_t);
	uint8_t forced_neighbors_fast_3d(uint8_t);

	void generate_lookup_table();
}

#endif

