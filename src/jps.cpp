#include "jps/jps.hpp"

using namespace std;
using namespace JPS;

pair<vector<Point>, float> JPS::jps(Point start, Point goal, bool (*is_valid)(const Point &)) {

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
  for (int8_t dx = -1; dx != 2; ++dx) {
    for (int8_t dy = -1; dy != 2; ++dy) {
      for (int8_t dz = -1; dz != 2; ++dz) {
        const Dir d(dx, dy, dz);
        if (d != 13 && is_valid(start + d)) {
          parents.insert({start+d, start});
          astar.push({start+d, d.norm()});
        }
      }
    }
  } 

  bool found = false;
  int astar_iter = 0;
  while (!found && !astar.empty()) {

    auto[c_node, d_start_current] = astar.top();
    astar.pop();
    const Point & c_parent(parents.find(c_node)->second);

    cout << "Currently at node " << c_node << " distance " << d_start_current << " coming from " << c_parent << endl;

    if (true) {
      // use Jump Point expansion 

      // parent in local 3x3 frame centered at c_node
      Dir d_parent(c_parent.x() - c_node.x(), c_parent.y() - c_node.y(), c_parent.z() - c_node.z());

      // obstacles in local 3x3 frames
      set<Dir> obstacles;
      for (int8_t dx = -1; dx != 2; ++dx) {
        for (int8_t dy = -1; dy != 2; ++dy) {
          for (int8_t dz = -1; dz != 2; ++dz) {
            const Dir d(dx, dy, dz);
            if (!is_valid(c_node + d)) {
              obstacles.insert(d);
            }
          }
        }
      }

      for (Dir d : neighbors(d_parent, obstacles)) {
        if (c_node+d == goal) {
          found = true;
        }
        if (parents.find(c_node+d) == parents.end()) {
          cout << "adding node " << c_node+d << " with distance " << d_start_current + d.norm() <<  endl;
          astar.push({c_node+d, d_start_current + d.norm()});
          parents.insert({c_node+d, c_node});
        }
      }
    } else {
      // regular astar expansion

      for (int8_t dx = -1; dx != 2; ++dx) {
        for (int8_t dy = -1; dy != 2; ++dy) {
          for (int8_t dz = -1; dz != 2; ++dz) {
            const Point neighbor(c_node + Dir(dx, dy, dz));
            if (parents.find(neighbor) == parents.end() && is_valid(neighbor))  {
              parents.insert({neighbor, c_node});
              float d_c_neighbor = (c_node - neighbor).norm();
              astar.push({neighbor, d_start_current + d_c_neighbor});
            }
            if (neighbor == goal) {
              found = true;
              break;
            }
          }
        }
      }

    }

    ++astar_iter;
  }


  if (found) {

    cout << "Finished astar after " << astar_iter << " iterations" << endl;

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

set<Dir> JPS::neighbors(Dir parent, set<Dir> obstacles) {
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
