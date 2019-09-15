#include "jps/jps.hpp"

using namespace std;
using namespace JPS;

pair<vector<Point>, float> JPS::jps(Point start, Point goal) {

  // map that gives parents in search
  map<Point, Point> parents;

  // priority queue with euclidean distance to target
  auto astar_cmp = [goal] (const ASNode & n1, const ASNode & n2) {
    float d_n1_target = (n1.first - goal).norm();
    float d_n2_target = (n2.first - goal).norm();
    float d_start_n1 = n1.second;
    float d_start_n2 = n2.second;
    return d_start_n1 + d_n1_target > d_start_n2 + d_n2_target; 
  };
  priority_queue<ASNode, vector<ASNode>, decltype(astar_cmp)> astar(astar_cmp);
  
  astar.push({start, 0});

  bool found = false;
  int astar_iter = 0;
  while (!found && !astar.empty()) {

    auto[current_node, d_start_current] = astar.top();
    astar.pop();

    // Expand current_node
    for (int8_t dx = -1; dx != 2; ++dx) {
      for (int8_t dy = -1; dy != 2; ++dy) {
        for (int8_t dz = -1; dz != 2; ++dz) {
          const Point neighbor(current_node + Dir(dx, dy, dz));
          if (parents.find(neighbor) == parents.end())  {
            parents.insert({neighbor, current_node});
            float d_current_neighbor = (current_node - neighbor).norm();
            astar.push({neighbor, d_start_current + d_current_neighbor});
          }
          if (neighbor == goal) {
            found = true;
            break;
          }
        }
      }
    }
    ++astar_iter;
  }

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

set<Dir> JPS::expand(Dir parent, set<Dir> obstacles) {
  // return all nodes that require expansion when moving 
  // into unoccupied center of 3x3 box from parent

  set<Dir> ret;
  
  // 1. add all natural neighbors 
  ret.insert(-parent);

  if (parent.order() == 2) {
    if (parent.dx() == 0 || parent.dy() == 0)
      ret.insert(Dir(0, 0, -parent.dz()));
    if (parent.dx() == 0 || parent.dz() == 0)
      ret.insert(Dir(0, -parent.dy(), 0));
    if (parent.dy() == 0 || parent.dz() == 0)
      ret.insert(Dir(-parent.dx(), 0, 0));
  } 

  if (parent.order() == 3) {
    ret.insert(Dir(-parent.dx(), -parent.dy(), 0));
    ret.insert(Dir(-parent.dx(), 0, -parent.dz()));
    ret.insert(Dir(0, -parent.dy(), -parent.dz()));
    ret.insert(Dir(-parent.dx(), 0, 0));
    ret.insert(Dir(0, -parent.dy(), 0));
    ret.insert(Dir(0, 0, -parent.dz()));
  }

  // remove obstacles if we got some
  set<Dir> c;
  set_difference(make_move_iterator(ret.begin()), 
                 make_move_iterator(ret.end()), 
                 obstacles.begin(), obstacles.end(), 
                 inserter(c, c.begin()));
  ret.swap(c);

  if (!obstacles.empty()) {
    // 2. add points for which path through center 
    // is shorter than shortest path that excludes center

    // solve djikstra from parent excluding obstacles and origin (27)
    map<Dir, float> dist;
    vector<Dir> remaining;
    for (int i=0; i!=27; ++i) {
      if (i != 13 && obstacles.find(i) == obstacles.end()) {
        dist[i] = 10;
        remaining.push_back(i);
      }
    }
    dist[parent] = 0;

    while (!remaining.empty()) {
      sort(remaining.begin(), remaining.end(), [dist] (const Dir & i1, const Dir & i2) {return dist.at(i1) > dist.at(i2);});
      Dir u(remaining.back());
      remaining.pop_back();
      for (int i=max(u.dx()-1, -1); i<min(2, u.dx()+2); ++i) {
        for (int j=max(u.dy()-1, -1); j<min(2, u.dy()+2); ++j) {
          for (int k=max(u.dz()-1, -1); k<min(2, u.dz()+2); ++k) {
            Dir neighbor(i, j, k);
            if (find(remaining.begin(), remaining.end(), neighbor) != remaining.end()) {
              dist[neighbor] = min(dist[neighbor], dist[u] + neighbor.distance_to(u));
            }
          }
        }
      }
    }

    for (auto[neigh, d] : dist) {
      if (parent.norm() + Dir(neigh).norm() < d) {
        ret.insert(Dir(neigh));
      }
      // cout << "Distance to " << Dir(neigh) << ": " << d << endl;
    }

  }

  return move(ret);
}
