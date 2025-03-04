- $cmp_f \longrightarrow$ floating point comparison
- $add_f \longrightarrow$ floating point addition
- $mul_f \longrightarrow$ floating point multiplication
- $div_f \longrightarrow$ floating point division
- $abs_f \longrightarrow$ absolute value
- $dot_v \longrightarrow$ dot product (3D)
- $x_v \longrightarrow$ cross product
- $add_v \longrightarrow$ vector addition

<div style="page-break-after: always;"></div>

# Octree
## Construction
<img src="./pngs/OBB_octree_edge_cases.png" width="600"/>

- Check the relative position of the OBB representing a region with the AABB representing an octree cell:
  - AABB fully outside
  - Intersection
  - AABB fully inside (intersection)
- Do this for all the regions of the cell:
  - **AABB fully inside OR fully outside all regions**: cell is a leaf, save regions where the AABB is fully inside.
  - **At least one region intersects (not fully inside)**: recursively subdivide the cell (only consider the regions that intersect it).
- Do this until cells are too small or all cells are leaves.
- To check the relative position of an AABB and an OBB we must use the separating axis theorem (SAT), which is complex.

$C_{construction}(root) = C_{intersection} + 8 \cdot C_{construction}(subtree)$

$C_{intersection} = C_{SAT} + C_{fully \space inside}$ <sub>early out if SAT fails, but little gains (SAT is way the most complex part by far)</sub>

$C_{fully \space inside} = 8 \cdot C_{OBB}$ <sub>check *AABB for OBB*: we check if all the vertices of the AABB are inside the OBB of the region</sub>

$C_{SAT} = 9 \cdot x_v + (6+6+9) \cdot [(8+8) \cdot (dot_v + 2 \cdot cmp_f)]$ <sub>approximate cost and many early outs</sub>

## Traversal
- Find the octree cell of the point and look up the regions.
- The search in the octree is linear w.r.t. the max number of levels allowed for the octree.

$C_{lookup} = levels \cdot [3 \cdot (mul_f + cmp_f)]$ <sub>for each axis we have to get if the component is before or after the half point</sub>

<img src="./pngs/OBB_octree.png" width="400"/>
<div style="page-break-after: always;"></div>

# AABB for OBB
## Construction
- For each vertex of each triangle, test if it is inside each BVH region.
- To check if it is inside:
  1. Check if a point is inside the AABB.
  2. Check if a point is inside the OBB:
     - To do so, we basically have to transform the point in the coordinate system of the OBB. 

$C_{construction} = (3 \cdot V \cdot BVHs) \cdot C_{inside}$

$C_{inside} = C_{AABB} + C_{OBB}$ <sub>early out if the AABB part fails</sub>

$C_{AABB} = 6 \cdot cmp_f$ <sub>*early out when one component doesn't satisfy the comparison (outside AABB)*</sub>

$C_{OBB} = add_v + 3 \cdot (abs_v + dot_v + cmp_f)$ <sub>*there may be an early out if the first (or second) coordinate is already out of the OOBB*</sub>

<img src="./pngs/OBB_AABB.png" width="500"/>

## Traversal
- For each BVH region, we have to check if the point is inside.

$C_{traversal} = BVHs \cdot C_{inside}$

<div style="page-break-after: always;"></div>

# Simplify to AABBs
<img src="./pngs/simplify_OBB.png" width="400"/>
