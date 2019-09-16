#include "jps/jps.hpp"

using namespace std;
using namespace JPS;

pair<vector<Point>, float> JPS::jps(Point start, Point goal, function<bool(const Point &)> is_valid) {

  // map that gives parents in search
  map<Point, Point> parents;

  // priority queue with euclidean distance to target
  auto astar_cmp = [goal] (const ASNode & n1, const ASNode & n2) {
    float d_n1_goal = (n1.first - goal).norm();
    float d_n2_goal = (n2.first - goal).norm();
    float d_start_n1 = n1.second;
    float d_start_n2 = n2.second;
    return d_start_n1 + d_n1_goal > d_start_n2 + d_n2_goal; 
  };
  priority_queue<ASNode, vector<ASNode>, decltype(astar_cmp)> astar(astar_cmp);
  
  // initialize with all valid neighbors around start
  parents.insert({start, start});
  astar.push({start, 0});

  bool found = false;
  int astar_iter = 0;
  while (!found && !astar.empty()) {


    auto[c_node, d_start_current] = astar.top();
    astar.pop();
    const Point & c_parent(parents.find(c_node)->second);

    cout << "In node " << c_node << " coming from " << d_start_current << " with distance " << d_start_current << endl;
    if (true) {
      // use Jump Point expansion
      for (Point p : successors(c_node, c_parent, goal, is_valid)) {
        if (p == goal)
          found = true;
        if (parents.find(p) != parents.end())
          throw string("JPS came back to visited node!");

        astar.push({p, (p - c_node).norm()});
        parents.insert({p, c_node});
      }
    } else {
      // regular astar expansion
      for (Dir d : NEIGHBORS_3D) {
        const Point neighbor(c_node + d);
        if (is_valid(neighbor) && parents.find(neighbor) == parents.end())  {
          parents.insert({neighbor, c_node});
          astar.push({neighbor, d_start_current + (c_node - neighbor).norm()});
        }
        if (neighbor == goal) {
          found = true;
          break;
        }
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

pair<bool, Point> JPS::jump(const Point & p, const Dir & d, const Point & goal, function<bool(const Point &)> is_valid) {

  if (!is_valid(p))  // can't jump from here
    return {false, p};

  Point par_iter = p;
  Point nod_iter = p+d;

  cout << "Jumping from " << p << " in direction " << d << endl;

  while(is_valid(nod_iter)) {     

    // check if we hit goal node
    if (nod_iter == goal)
      return {true, nod_iter};

    // check if forced neighbor exists
    if (!forced_neighbors(nod_iter, par_iter, is_valid).empty())
      return {true, nod_iter};

    // jump in lower-order directions
    if (d.order() == 2) {  // 2D Jump: check 1D
      if (d.dx() == 0) {
        auto[succ1, point1] = jump(nod_iter, Dir(0, d.dy(), 0), goal, is_valid);
        auto[succ2, point2] = jump(nod_iter, Dir(0, 0, d.dz()), goal, is_valid);
        if (succ1 || succ2)
          return {true, nod_iter};
      }
      if (d.dy() == 0) {
        auto[succ1, point1] = jump(nod_iter, Dir(d.dx(), 0, 0), goal, is_valid);
        auto[succ2, point2] = jump(nod_iter, Dir(0, 0, d.dz()), goal, is_valid);
        if (succ1 || succ2)
          return {true, nod_iter};
      }
      if (d.dz() == 0) {
        auto[succ1, point1] = jump(nod_iter, Dir(d.dx(), 0, 0), goal, is_valid);
        auto[succ2, point2] = jump(nod_iter, Dir(0, d.dy(), 0), goal, is_valid);
        if (succ1 || succ2)
          return {true, nod_iter};
      }
    }

    if (d.order() == 3) {  // 3D jump, check 2D and 1D
      auto[succ1, point1] = jump(nod_iter, Dir(d.dx(), d.dy(), 0), goal, is_valid);
      auto[succ2, point2] = jump(nod_iter, Dir(d.dx(), 0, d.dz()), goal, is_valid);
      auto[succ3, point3] = jump(nod_iter, Dir(0, d.dy(), d.dz()), goal, is_valid);
      auto[succ4, point4] = jump(nod_iter, Dir(d.dx(), 0, 0), goal, is_valid);
      auto[succ5, point5] = jump(nod_iter, Dir(0, d.dy(), 0), goal, is_valid);
      auto[succ6, point6] = jump(nod_iter, Dir(0, 0, d.dz()), goal, is_valid);

      if (succ1 || succ2 || succ3 || succ4 || succ5 || succ6)
        return {true, nod_iter};
    }

    par_iter = par_iter + d;
    nod_iter = nod_iter + d;
  }
  return {false, p};
}

std::set<Point> JPS::successors(const Point & node, const Point & parent, const Point & goal, function<bool(const Point &)> is_valid) {

  set<Point> ret;

  if (node == parent) {
    // jump in all possible directions
    for (Dir d : JPS::NEIGHBORS_3D) {
      auto[succ, new_node] = jump(node, d, goal, is_valid);
      if (succ) 
        ret.insert(new_node);
    }
  } else {
    // jump in neighbor directions
    for (Dir d : all_neighbors(node, parent, is_valid)) {
      auto[succ, new_node] = jump(node, d, goal, is_valid);
      if (succ) 
        ret.insert(new_node);
    }
  }
  return move(ret);
}

set<Dir> JPS::natural_neighbors(const Point & node, const Point & parent, function<bool(const Point &)> is_valid) {
  function<bool(const Dir &)> dir_is_valid = [node, is_valid] (const Dir & dir) {return is_valid(node + dir);};
  return JPS::natural_neighbors_(node.incoming_dir(parent), dir_is_valid);
}

set<Dir> JPS::forced_neighbors(const Point & node, const Point & parent, function<bool(const Point &)> is_valid) {
  function<bool(const Dir &)> dir_is_valid = [node, is_valid] (const Dir & dir) {return is_valid(node + dir);};
  return JPS::forced_neighbors_fast(node.incoming_dir(parent), dir_is_valid);
}

set<Dir> JPS::all_neighbors(const Point & node, const Point & parent, function<bool(const Point &)> is_valid) {
  function<bool(const Dir &)> dir_is_valid = [node, is_valid] (const Dir & dir) {return is_valid(node + dir);};
  return JPS::all_neighbors_(node.incoming_dir(parent), dir_is_valid);
}

set<Dir> JPS::natural_neighbors_(const Dir & parent, function<bool(const Dir &)> is_valid) {
  set<Dir> ret;
  
  ret.insert(-parent);

  if (parent.order() == 2) {
    if ((parent.dx() == 0 || parent.dy() == 0) && is_valid(Dir(0, 0, -parent.dz())))
      ret.insert(Dir(0, 0, -parent.dz()));
    if ((parent.dx() == 0 || parent.dz() == 0) && is_valid(Dir(0, -parent.dy(), 0)))
      ret.insert(Dir(0, -parent.dy(), 0));
    if ((parent.dy() == 0 || parent.dz() == 0) && is_valid(Dir(-parent.dx(), 0, 0)))
      ret.insert(Dir(-parent.dx(), 0, 0));
  } 

  if (parent.order() == 3) {
    if (is_valid(Dir(-parent.dx(), -parent.dy(), 0)))
      ret.insert(Dir(-parent.dx(), -parent.dy(), 0));
    if (is_valid(Dir(-parent.dx(), 0, -parent.dz())))
      ret.insert(Dir(-parent.dx(), 0, -parent.dz()));
    if (is_valid(Dir(0, -parent.dy(), -parent.dz())))
      ret.insert(Dir(0, -parent.dy(), -parent.dz()));
    if (is_valid(Dir(-parent.dx(), 0, 0)))
      ret.insert(Dir(-parent.dx(), 0, 0));
    if (is_valid(Dir(0, -parent.dy(), 0)))
      ret.insert(Dir(0, -parent.dy(), 0));
    if (is_valid(Dir(0, 0, -parent.dz())))
      ret.insert(Dir(0, 0, -parent.dz()));
  }

  return move(ret);
}

set<Dir> JPS::forced_neighbors_(const Dir & parent, function<bool(const Dir &)> is_valid) {

  set<Dir> ret;

  // define valid nodes in the box
  map<Dir, float> dist;
  vector<Dir> remaining;
  for (Dir d : JPS::NEIGHBORS_3D) {
    if (is_valid(d)) {        
      dist[d] = 10;
      remaining.push_back(d);
    }
  }
  dist[parent] = 0;

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
    if (parent.norm() + neigh.norm() < d) 
      ret.insert(Dir(neigh));

  // remove natural neighbors
  set<Dir> nat = JPS::natural_neighbors_(parent, is_valid);
  set<Dir> c;
  set_difference(make_move_iterator(ret.begin()), 
                 make_move_iterator(ret.end()), 
                 nat.begin(), nat.end(), 
                 inserter(c, c.begin()));
  ret.swap(c);

  return move(ret);
}

set<Dir> JPS::all_neighbors_(const Dir & parent, function<bool(const Dir &)> is_valid) {
  // return all nodes that require expansion when moving 
  // into unoccupied center of 3x3 box from parent

  set<Dir> ret, nat, forc;

  nat = JPS::natural_neighbors_(parent, is_valid);
  forc = JPS::forced_neighbors_fast(parent, is_valid);

  set_union(make_move_iterator(nat.begin()), make_move_iterator(nat.end()), 
            make_move_iterator(forc.begin()), make_move_iterator(forc.end()), 
            inserter(ret, ret.begin()));

  return move(ret);
}

set<Dir> JPS::forced_neighbors_fast(const Dir & parent, function<bool(const Dir &)> is_valid) {

  set<Dir> ret;
  auto coord_change = standardize_dir(parent);

  //  local frame
  function<bool(const Dir &)> is_valid_local = [is_valid, coord_change] (const Dir & d) {
    return is_valid(coord_change(d));
  };

  if (parent.order() == 1)
    ret = decode_fn_1d(lookup1d[encode_obs_1d(is_valid_local)]);

  if (parent.order() == 2)
    ret = decode_fn_2d(lookup2d[encode_obs_2d(is_valid_local)]);

  if (parent.order() == 3)
    ret = decode_fn_3d(lookup3d[encode_obs_3d(is_valid_local)]);

  // go back to regular frame and remove forced neighbors that are covered by obstacles
  set<Dir> ret_trans;
  for (Dir d : ret) 
    if (is_valid(coord_change(d)))
      ret_trans.insert(coord_change(d));

  return move(ret_trans);
}

void JPS::generate_lookup_table() {
  ofstream datafile;
  datafile.open ("lookup_table.cpp");

  // 1D problems
  datafile << "uint8_t const JPS::lookup1d[256] = {";
  for (uint16_t iter=0; iter!=256; ++iter) {
    set<Dir> obstacles = decode_obs_1d(uint8_t(iter));
    set<Dir> forced = forced_neighbors_(Dir(-1,0,0), [obstacles] (const Dir & d) {return obstacles.find(d) == obstacles.end();});
    uint8_t neighbors = encode_fn_1d([forced] (const Dir & d) {return forced.find(d) != forced.end();});
    datafile << (int) neighbors;
    if (iter < 255)
      datafile << ", ";
  }
  datafile << "};" << endl;
  // 2D problems
  datafile << "uint16_t const JPS::lookup2d[256] = {";  
  for (uint16_t iter=0; iter!=256; ++iter) {
    set<Dir> obstacles = decode_obs_2d(uint8_t(iter));
    set<Dir> forced = forced_neighbors_(Dir(-1,-1,0), [obstacles] (const Dir & d) {return obstacles.find(d) == obstacles.end();});
    uint16_t neighbors = encode_fn_2d([forced] (const Dir & d) {return forced.find(d) != forced.end();});
    datafile << (int) neighbors;
    if (iter < 255)
      datafile << ", ";
  }
  datafile << "};" << endl;  
  // 3D problems
  datafile << "uint16_t const JPS::lookup3d[64] = {";    
  for (uint16_t iter=0; iter!=64; ++iter) {
    set<Dir> obstacles = decode_obs_3d(uint8_t(iter));
    set<Dir> forced = forced_neighbors_(Dir(-1,-1,-1), [obstacles] (const Dir & d) {return obstacles.find(d) == obstacles.end();});
    uint16_t neighbors = encode_fn_3d([forced] (const Dir & d) {return forced.find(d) != forced.end();});
    datafile << (int) neighbors;
    if (iter < 63)
      datafile << ", ";
  }
  datafile << "};" << endl;  

  datafile.close();
}
