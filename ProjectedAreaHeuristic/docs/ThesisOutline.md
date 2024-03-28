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

---

# BVH properties
Name | Description
-|-
maxLeafCost | If a node has a cost (PAH or SAH depending on the cost strategy used) less than this, it is a leaf
maxLeafArea | If a node has an area (projected or surface depending on the cost strategy used) less than this, it is a leaf
maxLeafHitProbability | If the hit probability of this node is less than this, it is a leaf
maxTrianglesPerLeaf | If a node has fewer triangles than this, it is a leaf
maxLevels | If a node is at a level higher than this, it is a leaf
bins | How many splits to try to split a node into its children. A higher value makes more accurate BVHs, but is is also more expensive
maxNonFallbackLevels | If a node is at a level higher than this, the specified fallback strategies will be used. This can be used to avoid using an expensive strategy (such as PAH) even at deep levels, where there is less to gain. A way to tune this value may be to look at the overlapping percentage at each level of the BVH.
splitPlaneQualityThreshold | [0, 1]. If the quality of the split plane is less than this value, 2 things can happen: if a satisfactory split plane has already been found, use it; if not, use the fallback strategy to find the splitting plane. A low value will let the algorithm to find the best splitting plane more times, but will slow the construction down.
acceptableChildrenFatherHitProbabilityRatio | Defined as $\frac{leftChildHit\% + rightChildHit\%}{fatherHit\%}$. This value is used to determine if a splitting plane cut is acceptable when no more planes with the minimum quality are present. The value is used to approximate the overlapping of the 2 children nodes: in the ideal case (no overlapping) this value is less than 1. A low value forces the algorithm to use the fallback strategy more often, therefore will also slow the construction down.
excellentChildrenFatherHitProbabilityRatio | Same as above, but in this case this value is compared with the best children found after each splitting plane cut: if such value is lower than this, no more splitting planes are tried, even if they had the required quality.