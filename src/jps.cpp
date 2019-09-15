#include "jps.hpp"

Dir::Dir(uint8_t data) : data(data) {}

Dir::operator int8_t() const {return data;}

Dir::Dir(int8_t dx, int8_t dy, int8_t dz) : data(dx+1 + 3*(dy+1) + 9*(dz+1)) {}

int8_t Dir::dx() const {return data % 3 - 1;}

int8_t Dir::dy() const {return (data/3)%3 - 1;}

int8_t Dir::dz() const {return (data/9)%3 - 1;}

uint8_t Dir::order() const {
  return uint8_t((data % 3) != 1) + uint8_t(((data/3)%3) != 1) + uint8_t(((data/9)%3) != 1);
}

Dir Dir::operator-() {
  return Dir(-dx(),-dy(),-dz());
}

float Dir::distance_to(const Dir & other) const {
  return sqrt(float((dx() - other.dx())*(dx() - other.dx()) + 
                    (dy() - other.dy())*(dy() - other.dy()) +
                    (dz() - other.dz())*(dz() - other.dz())));
}

float Dir::norm() const {
  return sqrt(float(dx()*dx() + dy()*dy() + dz()*dz()));
}

ostream& operator<<(ostream& stream, const Dir& dir) {
    stream << "[" << (int) dir.dx() << "," << (int) dir.dy() << "," << (int) dir.dz() << "]";
    return stream;
}

set<Dir> expand(Dir parent, set<Dir> obstacles) {
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
    map<Dir, double> dist;
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
