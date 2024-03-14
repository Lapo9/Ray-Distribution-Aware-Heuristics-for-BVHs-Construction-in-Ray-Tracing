# Ray Distribution Aware Bounding Volume Hierarchies
## State of the art
### Ray tracing in general
#### Variance reduction techniques for ray tracing
### BVHs in general
#### SAH heuristic and its results
#### Other proposed heuristics (in particular ray distribution aware ones)

## Projected area heuristic PAH
### Different types of ray distributions and their projections
#### Technique for parallel ray distribution
#### Technique for point ray distribution

## Splitting plane facing technique

## Multiple influence areas and top-level structures
### AABB for OBB
### Octree

## Tests
### Test scenes
### Influence areas orientations and types
### Single vs multi influence areas
### Full scene vs partial scene influence areas

## Results
### PAH vs SAH estimate accuracy
### PAH with plane facing vs PAH vs SAH traversal metrics
#### When is it worth it to use PAH
### PAH vs SAH construction metrics
#### PAH cost computation
#### Plane facing computation
### PAH vs SAH memory consumption

## Conclusions
### Future developments

## Appendices
### The C++ test project structure