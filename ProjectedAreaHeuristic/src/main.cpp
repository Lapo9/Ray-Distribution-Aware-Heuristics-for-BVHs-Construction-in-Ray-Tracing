#include "Bvh.h"
#include "InfluenceArea.h"
#include "Utilities.h"
#include "BvhAnalyzer.h"
#include "TopLevel.h"
#include "TopLevelAnalyzer.h"
#include "AnalyzerActions.h"
#include "RayCaster.h"
#include "TestScene.h"
#include "distributions.h"

#include <iostream>
#include <memory>

using namespace std;
using namespace pah;
using namespace nlohmann;

int main() {
	//generate triangles
	mt19937 rng{ 1 };

	//generate triangles for different scenes
	distributions::UniformBoxDistribution mainDistribution3d{ 0,10, 0,10, 0,10 };
	distributions::UniformBoxDistribution otherDistribution3d{ -1,1, -1,1 , -1,1 };
	auto randomTriangles = Triangle::generateRandom(1000, rng, mainDistribution3d, otherDistribution3d);
	auto woodTriangles = Triangle::fromObj("D:/Users/lapof/Documents/Development/ProjectedAreaHeuristic/ProjectedAreaHeuristic/exampleData/WoodScene.obj");
	auto SuzanneTriangles = Triangle::fromObj("D:/Users/lapof/Documents/Development/ProjectedAreaHeuristic/ProjectedAreaHeuristic/exampleData/MonkeyModel.obj");

	//Top level analyzer used by most scenes
	TopLevelOctreeAnalyzer topLevelAnalyzer{
		MAKE_ACTIONS_PAIR(core),
		MAKE_ACTIONS_PAIR(sah),
		MAKE_ACTIONS_PAIR(pah),
		MAKE_ACTIONS_PAIR(levelCount),
		MAKE_ACTIONS_PAIR(triangles),
		MAKE_ACTIONS_PAIR(influenceArea),
		MAKE_ACTIONS_PAIR(timeMeasurement)
	};

	//properties for most BVHs
	Bvh::Properties bvhProperties{
	.maxLeafCost = 0.01f,
	.maxTrianglesPerLeaf = 2,
	.maxLevels = 100,
	.bins = 40,
	.splitPlaneQualityThreshold = 0.1f,
	.maxChildrenFatherHitProbabilityRatio = 1.3f
	};

	//top level structure properties used in most scenes
	TopLevel::Properties topLevelProperties{
	.splitPlaneQualityThreshold = 0.1f,
	.maxChildrenFatherHitProbabilityRatio = 1.3
	};

	//fallback BVH used by most top level structures
	Bvh fallbackBvh{ bvhProperties, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesLongest<0.5f>, bvhStrategies::shouldStopThresholdOrLevel, "fallback" };


	// === Wood scene: 1 plane influence area parallel to x covering all the scene. 1 ray caster relative to the influence area. ===
	PlaneInfluenceArea woodPlaneFullParallelArea{ Plane{{2.5,.8,0}, {-1,0,0}}, {1.5,1}, 20, 10000 };
	PlaneRayCaster woodPlaneFullParallelCaster{ woodPlaneFullParallelArea }; woodPlaneFullParallelCaster.generateRays(rng, 1000, true);
	Bvh woodPlaneFullParallelBvh{ bvhProperties, woodPlaneFullParallelArea, bvhStrategies::computeCostPah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane"};
	TopLevelOctree woodPlaneFullParallelTls{ topLevelProperties, TopLevelOctree::OctreeProperties{ 4, false }, fallbackBvh, std::move(woodPlaneFullParallelBvh) };
	auto woodPlaneFullParallelTls = TestScene{ "D:/Users/lapof/Documents/Development/ProjectedAreaHeuristic/Results/WoodPlaneFullParallel", woodTriangles, vector{&woodPlaneFullParallelCaster}, std::move(woodPlaneFullParallelTls), topLevelAnalyzer };

	//create influence areas
	PlaneInfluenceArea planeInfluenceArea1{ Plane{{-1.5,0,-1.5}, {1,0,1}}, {.5,.5}, 4, 10000 };
	PointInfluenceArea pointInfluenceArea1{ Pov{{0,.25,-2}, {0,0,1}, 30, 30}, 4, 0.5, 10000 };
	PointInfluenceArea pointInfluenceArea2{ Pov{{0,0,0}, {1,1,0}, 90, 60}, 8, 0.5, 100 };

	//build the ray casters (will later be used to cast rays)
	PlaneRayCaster planeRayCaster1{ planeInfluenceArea1 }; planeRayCaster1.generateRays(rng, 1000, true);
	PointRayCaster pointRayCaster1{ pointInfluenceArea1 }; pointRayCaster1.generateRays(rng, 1000, true);
	PointRayCaster pointRayCaster2{ pointInfluenceArea2 }; pointRayCaster2.generateRays(rng, 1000, true);

	Bvh bvh1{ bvhProperties, planeInfluenceArea1, bvhStrategies::computeCostPah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane1"};
	Bvh bvh3{ bvhProperties, pointInfluenceArea1, bvhStrategies::computeCostPah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "point1"};
	Bvh bvh4{ bvhProperties, pointInfluenceArea2, bvhStrategies::computeCostPah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "point2"};
	

	return 0;
}
