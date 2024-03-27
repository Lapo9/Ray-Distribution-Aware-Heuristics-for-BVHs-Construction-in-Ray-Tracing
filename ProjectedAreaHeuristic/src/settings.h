/** @file */
#pragma once

#define TIMING 1 /**< When is set, during the runtime there will be routines to collect detailed data about performance. Such routines are the one inside a @p TIME(body) macro.*/
#define RELEASE 0 /**< When is set, any routine that is not necessary (even the ones controlled by the @p TIMING switch), will be removed. Such routines are the one inside a @p INFO(body) or @p TIME(body) macro.*/

#if RELEASE //with RELEASE, we strip all not necessary routines
	#define INFO(body)
	#define TIME(body)
#else
	#define INFO(body) body
	#if TIMING //this controls detailed time-measurement related routines
		#define TIME(body) body
	#else
		#define TIME(body)
	#endif
#endif


#define TOLERANCE 0.01f /**< A small value that is used when a tolerance is needed. Sort of @p epsilon. */
#define NODE_COST 1.0f /**< The cost of a @p Ray intersecting an internal @p Bvh::Node. */
#define LEAF_COST 1.2f /**< The cost of a @p Ray intersecting a leaf @p Bvh::Node. */

#define DEFAULT_BVH_FALLBACK_STRATEGY_COMPUTE_COST bvhStrategies::computeCostSah
#define DEFAULT_BVH_FALLBACK_STRATEGY_SPLITTING_PLANE bvhStrategies::chooseSplittingPlanesLongest<0.f>
#define DEFAULT_BVH_FALLBACK_STRATEGY_SHOULD_STOP bvhStrategies::shouldStopThresholdOrLevel

#define FAST_ORTHOGRAPHIC_PROJECTIONS 0 /**< If true, the orthographic projections will project the points to the coordinate system of the projection plane, not to the canonical view volume (i.e. {[-1,-1], [1,1]}).  */

#define PAH_STRATEGY bvhStrategies::computeCostPahWithCulling
