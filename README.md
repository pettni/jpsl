# Jump Point Search Library

C++ implementation of the JPS path-finding algorithm in three dimensions.  

 - Flexible interface by implementing a custom ```bool state_valid(const JPSL::Point &)``` function
 - Non-forced neighbors are pruned by solving a local shortest-path problem

# Compilation and Installation

```
mkdir build && cd build
cmake ..
make
make test          # optional run tests 
sudo make install  # optional
```

# Roadmap

 - [x] Implement JPS
 - [x] Planning function that takes function ```state_valid()```
 - [x] Create a grid mask class that packs info into a ```uint16_t``` and uses it for obstacles and neighbors
 - [x] Symmetrize problem (i.e. always have parent in lower 2x2)
 - [x] Run all possible combinations of 3x3 problems, store solutions in lookup table
 - [ ] Implement ```state_valid()``` for Octomap
 - [ ] Optional buffer for ```state_valid()```
 - [ ] More testing to make sure fast neighbor code is correct
 - [ ] Make things iterators instead of returning ```std::set```
 - [ ] Easy switch between A*, JPS, Bounded JPS
 - [ ] Algos that are optimized for 2D problems
