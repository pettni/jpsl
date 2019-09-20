#include "jpsl/jpsl.hpp"
#include "jpsl/encodings.hpp"

using namespace std;
using namespace JPSL;

pair<vector<Point>, float> JPSL::plan(Point start, Point goal, const function<bool(const Point &)> & state_valid) {

  // map that remembers path
  map<Point, Point> parents;

  // priority queue with euclidean distance to target
  auto astar_cmp = [goal] (const ASNode & n1, const ASNode & n2) {
    float d_n1_goal = (get<0>(n1) - goal).norm();
    float d_n2_goal = (get<0>(n2) - goal).norm();
    float d_start_n1 = get<2>(n1);
    float d_start_n2 = get<2>(n2);
    return d_start_n1 + d_n1_goal > d_start_n2 + d_n2_goal; 
  };
  priority_queue<ASNode, vector<ASNode>, decltype(astar_cmp)> astar(astar_cmp);
  
  // initialize with all valid neighbors around start
  astar.push({start, start, 0});

  bool found = false;
  int astar_iter = 0;
  while (!found && !astar.empty()) {

    auto[node, parent, running_dist] = astar.top();
    astar.pop();

    parents.insert({node, parent});

    if (node == goal) {
      found = true;
      break;
    }

    if (true) { // use Jump Point expansion
      for (Point successor : successors(node, parent, goal, state_valid))
        if (parents.find(successor) == parents.end())
          astar.push({successor, node, running_dist + (successor - node).norm()});
    } else { // regular astar expansion
      for (Dir d : NEIGHBORS_3D) {
        const Point successor(node + d);
        if (state_valid(successor) && parents.find(successor) == parents.end())
          astar.push({successor, node, running_dist + (successor - node).norm()});
      }
    }
    ++astar_iter;
  }

  if (found) {
    vector<Point> sol_path;
    float cost = 0;
    while (goal != start) {
      sol_path.push_back(goal);
      goal = parents.find(goal)->second;
      cost += (goal - sol_path.back()).norm();
    }
    sol_path.push_back(goal);

    std::reverse(sol_path.begin(), sol_path.end());
    return {move(sol_path), cost};
  }
  return {vector<Point>(), -1};
}

std::set<Point> JPSL::successors(const Point & node, const Point & parent, const Point & goal, const function<bool(const Point &)> & state_valid) {

  set<Point> ret;

  const set<Dir> jump_dirs = (node==parent) 
    ? JPSL::NEIGHBORS_3D                       // no parent, all directions
    : all_neighbors(node, parent, state_valid);  // jump pruned directions

  for (Dir d : jump_dirs) {
    auto[succ, new_node] = jump(node, d, goal, state_valid);
    if (succ) 
      ret.insert(new_node);
  }

  return move(ret);
}

pair<bool, Point> JPSL::jump(const Point & p, const Dir & d, const Point & goal, const function<bool(const Point &)> & state_valid) {

  if (!state_valid(p))  // can't jump from here
    return {false, p};

  Point par_iter = p;
  Point nod_iter = p+d;

  while(state_valid(nod_iter)) {

    // check if we hit goal node
    if (nod_iter == goal)
      return {true, nod_iter};

    // check if forced neighbor exists
    if (has_forced_neighbor(nod_iter, par_iter, state_valid))
      return {true, nod_iter};

    // jump in lower-order directions
    if (d.order() == 2) {  // 2D Jump: check 1D
      if (d.dx() == 0) {
        auto[succ1, point1] = jump(nod_iter, Dir(0, d.dy(), 0), goal, state_valid);
        if (succ1)
          return {true, nod_iter};
        auto[succ2, point2] = jump(nod_iter, Dir(0, 0, d.dz()), goal, state_valid);
        if (succ2)
          return {true, nod_iter};
      }
      if (d.dy() == 0) {
        auto[succ1, point1] = jump(nod_iter, Dir(d.dx(), 0, 0), goal, state_valid);
        if (succ1)
          return {true, nod_iter};
        auto[succ2, point2] = jump(nod_iter, Dir(0, 0, d.dz()), goal, state_valid);
        if (succ2)
          return {true, nod_iter};
      }
      if (d.dz() == 0) {
        auto[succ1, point1] = jump(nod_iter, Dir(d.dx(), 0, 0), goal, state_valid);
        if (succ1)
          return {true, nod_iter};
        auto[succ2, point2] = jump(nod_iter, Dir(0, d.dy(), 0), goal, state_valid);
        if (succ2)
          return {true, nod_iter};
      }
    }

    if (d.order() == 3) {  // 3D jump, check 2D and 1D
      auto[succ1, point1] = jump(nod_iter, Dir(d.dx(), 0, 0), goal, state_valid);
      if (succ1)
        return {true, nod_iter};
      auto[succ2, point2] = jump(nod_iter, Dir(0, d.dy(), 0), goal, state_valid);
      if (succ2)
        return {true, nod_iter};
      auto[succ3, point3] = jump(nod_iter, Dir(0, 0, d.dz()), goal, state_valid);
      if (succ3)
        return {true, nod_iter};
      auto[succ4, point4] = jump(nod_iter, Dir(d.dx(), d.dy(), 0), goal, state_valid);
      if (succ4)
        return {true, nod_iter};
      auto[succ5, point5] = jump(nod_iter, Dir(d.dx(), 0, d.dz()), goal, state_valid);
      if (succ5)
        return {true, nod_iter};
      auto[succ6, point6] = jump(nod_iter, Dir(0, d.dy(), d.dz()), goal, state_valid);
      if (succ6)
        return {true, nod_iter};
    }

    par_iter += d;
    nod_iter += d;
  }
  return {false, p};
}

set<Dir> JPSL::natural_neighbors(const Point & node, const Point & parent, const function<bool(const Point &)> & state_valid) {
  set<Dir> ret;

  Dir par_dir = node.direction_to(parent);
  
  ret.insert(-par_dir);

  if (par_dir.order() == 2) {
    if ((par_dir.dx() == 0 || par_dir.dy() == 0) && state_valid(node + Dir(0, 0, -par_dir.dz())))
      ret.emplace(0, 0, -par_dir.dz());
    if ((par_dir.dx() == 0 || par_dir.dz() == 0) && state_valid(node + Dir(0, -par_dir.dy(), 0)))
      ret.emplace(0, -par_dir.dy(), 0);
    if ((par_dir.dy() == 0 || par_dir.dz() == 0) && state_valid(node + Dir(-par_dir.dx(), 0, 0)))
      ret.emplace(-par_dir.dx(), 0, 0);
  } 

  if (par_dir.order() == 3) {
    if (state_valid(node + Dir(-par_dir.dx(), -par_dir.dy(), 0)))
      ret.emplace(-par_dir.dx(), -par_dir.dy(), 0);
    if (state_valid(node + Dir(-par_dir.dx(), 0, -par_dir.dz())))
      ret.emplace(-par_dir.dx(), 0, -par_dir.dz());
    if (state_valid(node + Dir(0, -par_dir.dy(), -par_dir.dz())))
      ret.emplace(0, -par_dir.dy(), -par_dir.dz());
    if (state_valid(node + Dir(-par_dir.dx(), 0, 0)))
      ret.emplace(-par_dir.dx(), 0, 0);
    if (state_valid(node + Dir(0, -par_dir.dy(), 0)))
      ret.emplace(0, -par_dir.dy(), 0);
    if (state_valid(node + Dir(0, 0, -par_dir.dz())))
      ret.emplace(0, 0, -par_dir.dz());
  }

  return move(ret);
}

set<Dir> JPSL::all_neighbors(const Point & node, const Point & parent, const function<bool(const Point &)> & state_valid) {
  // return all nodes that require expansion when moving 
  // into unoccupied center of 3x3 box from parent

  set<Dir> ret, nat, forc;

  nat = JPSL::natural_neighbors(node, parent, state_valid);
  forc = JPSL::forced_neighbors_fast(node, parent, state_valid);

  set_union(make_move_iterator(nat.begin()), make_move_iterator(nat.end()), 
            make_move_iterator(forc.begin()), make_move_iterator(forc.end()), 
            inserter(ret, ret.begin()));

  return move(ret);
}

set<Dir> JPSL::forced_neighbors_fast(const Point & node, const Point & parent, const function<bool(const Point &)> & state_valid) {

  Dir d = node.direction_to(parent);
  set<Dir> ret;

  switch (d.order()) {
    case 1:
      return forced_neighbors_fast_1d(node, parent, state_valid);   // closed-form solution
      break;
    case 2:
      ret = decode_fn_2d(node, parent, lookup2d[encode_obs_2d(node, parent, state_valid)]);
      break;
    case 3:
      ret = decode_fn_3d(node, parent, lookup3d[encode_obs_3d(node, parent, state_valid)]);
      break;
  }

  set<Dir> ret_trans;
  for (Dir d : ret) 
    if (state_valid(node+d))
      ret_trans.insert(d);

  return move(ret_trans);
}

std::set<Dir> JPSL::forced_neighbors_fast_1d(const Point & node, const Point & parent, const std::function<bool(const Point &)> & state_valid) {
  Dir d_parent = node.direction_to(parent);

  set<Dir> ret;

  if (d_parent.dx() != 0) 
    for (Dir d_cand : NIEGHBORS1D_DX)
      if (!state_valid(node + d_cand) && state_valid(node + d_cand - d_parent))
        ret.emplace(-d_parent.dx(), d_cand.dy(), d_cand.dz());  

  if (d_parent.dy() != 0) 
    for (Dir d_cand : NIEGHBORS1D_DY)
      if (!state_valid(node + d_cand) && state_valid(node + d_cand - d_parent))
        ret.emplace(d_cand.dx(), -d_parent.dy(), d_cand.dz());  

  if (d_parent.dz() != 0) 
    for (Dir d_cand : NIEGHBORS1D_DZ)
      if (!state_valid(node + d_cand) && state_valid(node + d_cand - d_parent))
        ret.emplace(d_cand.dx(), d_cand.dy(), -d_parent.dz());  

  return move(ret);
}

bool JPSL::has_forced_neighbor(const Point & node, const Point & parent, const function<bool(const Point &)> & state_valid) {

  set<Dir> ret;

  switch (node.direction_to(parent).order()) {
    case 1:
      return has_forced_neighbor_fast_1d(node, parent, state_valid);   // closed-form solution
      break;
    case 2:
      ret = decode_fn_2d(node, parent, lookup2d[encode_obs_2d(node, parent, state_valid)]);
      for (Dir d : ret)
        if (state_valid(node+d))
          return true;
      break;
    case 3:
      ret = decode_fn_3d(node, parent, lookup3d[encode_obs_3d(node, parent, state_valid)]);
      for (Dir d : ret)
        if (state_valid(node+d))
          return true;
      break;
  }
  return false;
}

bool JPSL::has_forced_neighbor_fast_1d(const Point & node, const Point & parent, const std::function<bool(const Point &)> & state_valid) {
  Dir d_parent = node.direction_to(parent);

  set<Dir> ret;

  if (d_parent.dx() != 0) 
    for (Dir d_cand : NIEGHBORS1D_DX)
      if (!state_valid(node + d_cand) && state_valid(node + d_cand - d_parent))
        return true;  

  if (d_parent.dy() != 0) 
    for (Dir d_cand : NIEGHBORS1D_DY)
      if (!state_valid(node + d_cand) && state_valid(node + d_cand - d_parent))
        return true;  

  if (d_parent.dz() != 0) 
    for (Dir d_cand : NIEGHBORS1D_DZ)
      if (!state_valid(node + d_cand) && state_valid(node + d_cand - d_parent))
        return true;

  return false;
}

set<Dir> JPSL::forced_neighbors_slow(const Point & node, const Point & parent, const function<bool(const Point &)> & state_valid) {

  Dir d_parent = node.direction_to(parent);

  set<Dir> ret;

  // define valid nodes in the box
  map<Dir, float> dist;
  vector<Dir> remaining;
  for (Dir d : JPSL::NEIGHBORS_3D) {
    if (state_valid(node + d)) {        
      dist[d] = 10;
      remaining.push_back(d);
    }
  }
  dist[d_parent] = 0;

  // do djikstra
  while (!remaining.empty()) {
    sort(remaining.begin(), remaining.end(), [dist] (const Dir & i1, const Dir & i2) {return dist.at(i1) > dist.at(i2);});
    Dir u(remaining.back());
    remaining.pop_back();
    for (int i=max(u.dx()-1, -1); i<min(2, u.dx()+2); ++i)
      for (int j=max(u.dy()-1, -1); j<min(2, u.dy()+2); ++j)
        for (int k=max(u.dz()-1, -1); k<min(2, u.dz()+2); ++k)
          if (find(remaining.begin(), remaining.end(), Dir(i,j,k)) != remaining.end())
            dist[Dir(i,j,k)] = min(dist[Dir(i,j,k)], dist[u] + Dir(i,j,k).distance_to(u));
  }

  // forced neighbors are those whose shortest distance from parent is through (0,0,0)
  for (auto[neigh, d] : dist) 
    if (d_parent.norm() + neigh.norm() < d) 
      ret.insert(Dir(neigh));

  // remove natural neighbors
  set<Dir> nat = JPSL::natural_neighbors(node, parent, state_valid);
  set<Dir> c;
  set_difference(make_move_iterator(ret.begin()), 
                 make_move_iterator(ret.end()), 
                 nat.begin(), nat.end(), 
                 inserter(c, c.begin()));
  ret.swap(c);

  return move(ret);
}