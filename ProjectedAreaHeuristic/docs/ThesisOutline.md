# Ray Distribution Aware Heuristics for Bounding Volume Hierarchies Construction
## Abstract

## Introduction

## Background Theory
### Ray Tracing Principles
- What is ray tracing
- Advantages
- Disadvantages
- Used for off-line rendering
- How to overcome the disadvantages:
  - Used only for specific effects
  - GPU specialization
  - Monte-Carlo variance reduction techniques

### Monte-Carlo and Variance Reduction Techniques
- Kajiya's rendering equation (hemisphere integrals)
- We must use Monte-Carlo to approximate it
- Importance sampling:
  - Next event estimation
  - BRDF sampling
  - Multiple importance sampling
- The conclusion is that rays are not randomly distributed in the scene, but tend to follow certain patterns in certain regions

### Ray Tracing Acceleration Structures
- Why we need an acceleration structure
- Bounding volume hierarchies
- Axis aligned bounding boxes
- BVH construction algorithm overview
- Splitting plane
- BVH cost
- Surface area heuristic
- SAH 2 hypotheses:
  1. Rays start from outside the scene
  2. Rays are randomly distributed both in space end directions

## Projected Area Heuristic
### SAH hypotheses fall
- Hypothesis 1 fall and papers on it
- Hypothesis 2 fall:
  - A novel heuristic
### Different Types of Ray Distributions and Their Associated Projections
- General explanation of influence areas and regions
#### Parallel Rays Distribution
- Explanation of used techniques...
- Region encoded as OBB enclosed in AABB
- Parallelogram area
- Projection
#### Point Rays Distribution
- Explanation of used techniques...
- Region encoded as a frustum
- Contour points retrieval
- Projection

## Splitting Plane Facing Technique
- We want to avoid overlapping
- Related studies (EPO metric)
- Our algorithm and its parameters (speed vs quality)

## Multiple Influence Areas and Top-Level Structures
- In a real scene we will have more than a single influence area...
### AABB for OBB
- No construction overhead, very slow to traverse
### Octree
- Construction algorithm
- Traversal

## Experimental Results
### Test Scenes
- Which scenes and the characteristics of each one
### Influence Areas Orientations and Types
- Parallel, 15, 45, oblique
### PAH vs SAH Estimate Accuracy
- Explain how we calculate the real cost of a BVH
- Talk about the culling
### PAH with and without Plane Facing Splitting Plane Strategy
- Overlapping % (only on first levels)
- Try with different parameters
### When Is It Worth It to Use PAH
- Hit % and estimated hit % threshold
### PAH vs SAH construction metrics
#### PAH Cost Computation
#### Plane Facing Splitting Plane Strategy Computation
### PAH vs SAH Memory Consumption

- 

## Use Cases
- ???

## Conclusions and Future Developments

## Appendix A: Collision and Culling Algorithms
### Ray-AABB Collision
### Ray-Plane Collision
### AABB-AABB Collision
### 3D Convex Hull-3D Convex Hull Collision
### Point Inside AABB
### Point Inside Frustum
### Point Inside 2D Convex Hull
### 2D Hull Culling
## Appendix B: The C++ Test Project Structure

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

# Octree