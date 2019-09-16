# 3D Jump-Point Search

## Why this one?

 - Optimal pruning via pre-computation (WIP)
 - Flexible interface

## Compilation

```
mkdir build && cd build
cmake ..
make
```
Then run tests with ```ctest```

## TODOs

 - [x] Implement JPS
 - [x] Planning function that takes function ```state_valid()```
 - [x] Create a grid mask class that packs info into a ```uint16_t``` and uses it for obstacles and neighbors
 - [x] Symmetrize problem (i.e. always have parent in lower 2x2)
 - [x] Run all possible combinations of 3x3 problems, store solutions in lookup table
 - [ ] Implement ```state_valid()``` for octomaps
 - [ ] Optional buffer for ```state_valid()```
 - [ ] Optimize for 2D problems
