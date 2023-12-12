# [ ] State of the art
- [ ] BVH algorithms
- [ ] Sampling techniques
- [ ] Specific articles (e.g. contour integrals)

# [ ] Comparison algorithms
- [x] BVH with SAH
- [ ] Grids
- [ ] Octrees
- [ ] Binary space partitioning

# [ ] Novelty
- [ ] CPU
  - [ ] Clustering for influence area detection
  - [x] **Single area PAH**
    - [ ] Orthogonal optimization
    - [ ] Perspective optimization
  - [x] **Single area plane splitting**
    - [ ] *Does it work when there is no clear direction to choose from?*
    - [x] Choose more than one plane if results are similar
      - [ ] Tuning of multiple plane selection
  - [ ] **Multi area PAH**
    - [ ] *Deferred rendering integration?*
    - [ ] **Top level acceleration structure**
      - [ ] *How to create?*
      - [ ] Traversal (choose best BVH based on ray direction)
  - [ ] **Multi area plane splitting** 
  - [ ] **Data collection**
    - [ ] How to collect data
    - [ ] Time results
    - [ ] Functional results
- [x] **Port to C++**
- [x] Visualizer in Unity
- [ ] GPU
  - [ ] *What framework?*
  - [ ] Single area PAH
  - [ ] Single area plane splitting
  - [ ] Multi area PAH
  - [ ] Multi area plane splitting
 
# [ ] Results