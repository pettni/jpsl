# 3D Jump-Point Search

## Why this one?

 - Faster than Kumar's algorithm, prunes more points

## TODOs

 - Implement JPS
 - Planning function that takes function ```state_valid()```
 - Implement ```state_valid()``` for octomaps
 
 - Create a grid mask class that packs info into a ```uint32_t``` and uses it for obstacles and neighbors

 - Symmetrize problem (i.e. always have parent in lower 2x2)
 - Run all possible combinations of 3x3 problems, store solutions in header file
