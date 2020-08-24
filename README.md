# Jump Point Search Library

C++ implementation of the [jump-point search  path-finding algorithm](http://users.cecs.anu.edu.au/~dharabor/data/papers/harabor-grastien-aaai11.pdf) in three dimensions. JPS is an improvement for A* on uniform grids that can lead to dramatically reduced solution times.

 - Flexible interface via a custom ```bool state_valid(const JPSL::Point &)``` function, similar to [ompl](http://ompl.kavrakilab.org/)
 - Forced neighbors are identified by solving a local shortest-path problem, leading to maximal pruning
 - Solutions to local shortest-path problems are stored in a lookup table for speed

# Compilation and Installation

```
mkdir build && cd build
cmake ..
make
make test          # optional run tests 
sudo make install  # optional
```

# See also

 - https://github.com/KumarRobotics/jps3d
