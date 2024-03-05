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
	const vector randomTriangles = Triangle::generateRandom(100, rng, mainDistribution3d, otherDistribution3d);
	const vector woodTriangles = Triangle::fromObj("D:/Users/lapof/Documents/Development/ProjectedAreaHeuristic/ProjectedAreaHeuristic/exampleData/WoodScene.obj");
	const vector suzanneTriangles = Triangle::fromObj("D:/Users/lapof/Documents/Development/ProjectedAreaHeuristic/ProjectedAreaHeuristic/exampleData/MonkeyModel.obj");

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


	// Filter test scenes to run
	constexpr bool ALL = true;
	constexpr bool PLANE_FULL_PARALLEL = ALL || false;
	constexpr bool PLANE_FULL_PARALLEL_LONGEST = ALL || true;
	constexpr bool WOOD_SCENE = ALL || false;
	constexpr bool SUZANNE_SCENE = ALL || false;
	constexpr bool RANDOM_SCENE = ALL || false;
	constexpr string_view RESULTS_DIRECTORY = "D:/Users/lapof/Documents/Development/ProjectedAreaHeuristic/Results/";


	// === Wood scene: 1 plane full influence area ===
	if constexpr (PLANE_FULL_PARALLEL || WOOD_SCENE) {
		// === Wood scene: 1 plane influence area parallel to x covering all the scene. 1 ray caster relative to the influence area. ===
		{
			PlaneInfluenceArea influenceAreaPlaneFullParallel{ Plane{{2.5,.9,0}, {-1,0,0}}, {1.5,0.9}, 10, 10000 };
			PlaneRayCaster rayCasterPlaneFullParallel{ influenceAreaPlaneFullParallel }; rayCasterPlaneFullParallel.generateRays(rng, 1000, true);
			Bvh bvhPlaneFullParallel{ bvhProperties, influenceAreaPlaneFullParallel, bvhStrategies::computeCostPah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
			TopLevelOctree topLevel{ topLevelProperties, TopLevelOctree::OctreeProperties{ 4, false }, fallbackBvh, std::move(bvhPlaneFullParallel) };
			auto scene = TestScene{ string(RESULTS_DIRECTORY) + "WoodPlaneFullParallel", woodTriangles, vector{&rayCasterPlaneFullParallel}, std::move(topLevel), topLevelAnalyzer };
			scene.buildAndTraverse();
		}

		// === Wood scene: 1 plane influence area 15 degrees covering all the scene. 1 ray caster relative to the influence area. ===
		{
			PlaneInfluenceArea influenceAreaPlaneFull15{ Plane{{2.5,.8,-.3}, {-.966,0,.259}}, {1.6,1}, 10, 10000 };
			PlaneRayCaster rayCasterPlaneFull15{ influenceAreaPlaneFull15 }; rayCasterPlaneFull15.generateRays(rng, 1000, true);
			Bvh bvhPlaneFull15{ bvhProperties, influenceAreaPlaneFull15, bvhStrategies::computeCostPah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
			TopLevelOctree topLevel{ topLevelProperties, TopLevelOctree::OctreeProperties{ 4, false }, fallbackBvh, std::move(bvhPlaneFull15) };
			auto scene = TestScene{ string(RESULTS_DIRECTORY) + "WoodPlaneFull15", woodTriangles, vector{&rayCasterPlaneFull15}, std::move(topLevel), topLevelAnalyzer };
			scene.buildAndTraverse();
		}

		// === Wood scene: 1 plane influence area 45 degrees covering all the scene. 1 ray caster relative to the influence area. ===
		{
			PlaneInfluenceArea influenceAreaPlaneFull45{ Plane{{2.5,.8,-2.5}, {-1,0,1}}, {2,1}, 10, 10000 };
			PlaneRayCaster rayCasterPlaneFull45{ influenceAreaPlaneFull45 }; rayCasterPlaneFull45.generateRays(rng, 1000, true);
			Bvh bvhPlaneFull45{ bvhProperties, influenceAreaPlaneFull45, bvhStrategies::computeCostPah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
			TopLevelOctree topLevel{ topLevelProperties, TopLevelOctree::OctreeProperties{ 4, false }, fallbackBvh, std::move(bvhPlaneFull45) };
			auto scene = TestScene{ string(RESULTS_DIRECTORY) + "WoodPlaneFull45", woodTriangles, vector{&rayCasterPlaneFull45}, std::move(topLevel), topLevelAnalyzer };
			scene.buildAndTraverse();
		}

		// === Wood scene: 1 plane influence area oblique covering all the scene. 1 ray caster relative to the influence area. ===
		{
			PlaneInfluenceArea influenceAreaPlaneFullOblique{ Plane{{2.5,1.5,-1}, {-1,-0.3,0.4}}, {1.8,1.3}, 20, 10000 };
			PlaneRayCaster rayCasterPlaneFullOblique{ influenceAreaPlaneFullOblique }; rayCasterPlaneFullOblique.generateRays(rng, 1000, true);
			Bvh bvhPlaneFullOblique{ bvhProperties, influenceAreaPlaneFullOblique, bvhStrategies::computeCostPah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
			TopLevelOctree topLevel{ topLevelProperties, TopLevelOctree::OctreeProperties{ 4, false }, fallbackBvh, std::move(bvhPlaneFullOblique) };
			auto scene = TestScene{ string(RESULTS_DIRECTORY) + "WoodPlaneFullOblique", woodTriangles, vector{&rayCasterPlaneFullOblique}, std::move(topLevel), topLevelAnalyzer };
			scene.buildAndTraverse();
		}
	}

	// === Suzanne scene: 1 plane full influence area ===
	if constexpr (PLANE_FULL_PARALLEL || SUZANNE_SCENE) {
		// === Suzanne scene: 1 plane influence area parallel to x covering all the scene. 1 ray caster relative to the influence area. ===
		{
			PlaneInfluenceArea influenceAreaPlaneFullParallel{ Plane{{0,0,2}, {0,0,-1}}, {.9,.9}, 10, 10000 };
			PlaneRayCaster rayCasterPlaneFullParallel{ influenceAreaPlaneFullParallel }; rayCasterPlaneFullParallel.generateRays(rng, 1000, true);
			Bvh bvhPlaneFullParallel{ bvhProperties, influenceAreaPlaneFullParallel, bvhStrategies::computeCostPah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
			TopLevelOctree topLevel{ topLevelProperties, TopLevelOctree::OctreeProperties{ 4, false }, fallbackBvh, std::move(bvhPlaneFullParallel) };
			auto scene = TestScene{ string(RESULTS_DIRECTORY) + "SuzannePlaneFullParallel", suzanneTriangles, vector{&rayCasterPlaneFullParallel}, std::move(topLevel), topLevelAnalyzer };
			scene.buildAndTraverse();
		}

		// === Suzanne scene: 1 plane influence area 15 degrees covering all the scene. 1 ray caster relative to the influence area. ===
		{
			PlaneInfluenceArea influenceAreaPlaneFull15{ Plane{{.5,0,2}, {-.259,0,-.966}}, {1,.9}, 10, 10000 };
			PlaneRayCaster rayCasterPlaneFull15{ influenceAreaPlaneFull15 }; rayCasterPlaneFull15.generateRays(rng, 1000, true);
			Bvh bvhPlaneFull15{ bvhProperties, influenceAreaPlaneFull15, bvhStrategies::computeCostPah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
			TopLevelOctree topLevel{ topLevelProperties, TopLevelOctree::OctreeProperties{ 4, false }, fallbackBvh, std::move(bvhPlaneFull15) };
			auto scene = TestScene{ string(RESULTS_DIRECTORY) + "SuzannePlaneFull15", suzanneTriangles, vector{&rayCasterPlaneFull15}, std::move(topLevel), topLevelAnalyzer };
			scene.buildAndTraverse();
		}

		// === Suzanne scene: 1 plane influence area 45 degrees covering all the scene. 1 ray caster relative to the influence area. ===
		{
			PlaneInfluenceArea influenceAreaPlaneFull45{ Plane{{2,0,2}, {-1,0,-1}}, {1.2,.9}, 10, 10000 };
			PlaneRayCaster rayCasterPlaneFull45{ influenceAreaPlaneFull45 }; rayCasterPlaneFull45.generateRays(rng, 1000, true);
			Bvh bvhPlaneFull45{ bvhProperties, influenceAreaPlaneFull45, bvhStrategies::computeCostPah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
			TopLevelOctree topLevel{ topLevelProperties, TopLevelOctree::OctreeProperties{ 4, false }, fallbackBvh, std::move(bvhPlaneFull45) };
			auto scene = TestScene{ string(RESULTS_DIRECTORY) + "SuzannePlaneFull45", suzanneTriangles, vector{&rayCasterPlaneFull45}, std::move(topLevel), topLevelAnalyzer };
			scene.buildAndTraverse();
		}

		// === Suzanne scene: 1 plane influence area oblique covering all the scene. 1 ray caster relative to the influence area. ===
		{
			PlaneInfluenceArea influenceAreaPlaneFullOblique{ Plane{{1,-.5,2}, {-.45,.3,-1}}, {.9,.9}, 10, 10000 };
			PlaneRayCaster rayCasterPlaneFullOblique{ influenceAreaPlaneFullOblique }; rayCasterPlaneFullOblique.generateRays(rng, 1000, true);
			Bvh bvhPlaneFullOblique{ bvhProperties, influenceAreaPlaneFullOblique, bvhStrategies::computeCostPah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
			TopLevelOctree topLevel{ topLevelProperties, TopLevelOctree::OctreeProperties{ 4, false }, fallbackBvh, std::move(bvhPlaneFullOblique) };
			auto scene = TestScene{ string(RESULTS_DIRECTORY) + "SuzannePlaneFullOblique", suzanneTriangles, vector{&rayCasterPlaneFullOblique}, std::move(topLevel), topLevelAnalyzer };
			scene.buildAndTraverse();
		}
	}

	// === Random scene: 1 plane full influence area ===
	if constexpr (PLANE_FULL_PARALLEL || RANDOM_SCENE) {
		// === Random scene: 1 plane influence area parallel to x covering all the scene. 1 ray caster relative to the influence area. ===
		{
			PlaneInfluenceArea influenceAreaPlaneFullParallel{ Plane{{5,5,-4}, {0,0,1}}, {5,5}, 20, 10000 };
			PlaneRayCaster rayCasterPlaneFullParallel{ influenceAreaPlaneFullParallel }; rayCasterPlaneFullParallel.generateRays(rng, 1000, true);
			Bvh bvhPlaneFullParallel{ bvhProperties, influenceAreaPlaneFullParallel, bvhStrategies::computeCostPah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
			TopLevelOctree topLevel{ topLevelProperties, TopLevelOctree::OctreeProperties{ 4, false }, fallbackBvh, std::move(bvhPlaneFullParallel) };
			auto scene = TestScene{ string(RESULTS_DIRECTORY) + "RandomPlaneFullParallel", randomTriangles, vector{&rayCasterPlaneFullParallel}, std::move(topLevel), topLevelAnalyzer };
			scene.buildAndTraverse();
		}

		// === Random scene: 1 plane influence area 15 degrees covering all the scene. 1 ray caster relative to the influence area. ===
		{
			PlaneInfluenceArea influenceAreaPlaneFull15{ Plane{{3.5,5,-4}, {.259,0,.966}}, {6.5,5}, 20, 10000 };
			PlaneRayCaster rayCasterPlaneFull15{ influenceAreaPlaneFull15 }; rayCasterPlaneFull15.generateRays(rng, 1000, true);
			Bvh bvhPlaneFull15{ bvhProperties, influenceAreaPlaneFull15, bvhStrategies::computeCostPah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
			TopLevelOctree topLevel{ topLevelProperties, TopLevelOctree::OctreeProperties{ 4, false }, fallbackBvh, std::move(bvhPlaneFull15) };
			auto scene = TestScene{ string(RESULTS_DIRECTORY) + "RandomPlaneFull15", randomTriangles, vector{&rayCasterPlaneFull15}, std::move(topLevel), topLevelAnalyzer };
			scene.buildAndTraverse();
		}

		// === Random scene: 1 plane influence area 45 degrees covering all the scene. 1 ray caster relative to the influence area. ===
		{
			PlaneInfluenceArea influenceAreaPlaneFull45{ Plane{{-3,5,-3}, {1,0,1}}, {7,5}, 20, 10000 };
			PlaneRayCaster rayCasterPlaneFull45{ influenceAreaPlaneFull45 }; rayCasterPlaneFull45.generateRays(rng, 1000, true);
			Bvh bvhPlaneFull45{ bvhProperties, influenceAreaPlaneFull45, bvhStrategies::computeCostPah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
			TopLevelOctree topLevel{ topLevelProperties, TopLevelOctree::OctreeProperties{ 4, false }, fallbackBvh, std::move(bvhPlaneFull45) };
			auto scene = TestScene{ string(RESULTS_DIRECTORY) + "RandomPlaneFull45", randomTriangles, vector{&rayCasterPlaneFull45}, std::move(topLevel), topLevelAnalyzer };
			scene.buildAndTraverse();
		}

		// === Random scene: 1 plane influence area oblique covering all the scene. 1 ray caster relative to the influence area. ===
		{
			PlaneInfluenceArea influenceAreaPlaneFullOblique{ Plane{{10,12,-3}, {-.55,-.9,1}}, {6.5,6.5}, 20, 10000 };
			PlaneRayCaster rayCasterPlaneFullOblique{ influenceAreaPlaneFullOblique }; rayCasterPlaneFullOblique.generateRays(rng, 1000, true);
			Bvh bvhPlaneFullOblique{ bvhProperties, influenceAreaPlaneFullOblique, bvhStrategies::computeCostPah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
			TopLevelOctree topLevel{ topLevelProperties, TopLevelOctree::OctreeProperties{ 4, false }, fallbackBvh, std::move(bvhPlaneFullOblique) };
			auto scene = TestScene{ string(RESULTS_DIRECTORY) + "RandomPlaneFullOblique", randomTriangles, vector{&rayCasterPlaneFullOblique}, std::move(topLevel), topLevelAnalyzer };
			scene.buildAndTraverse();
		}
	}


	// === Wood scene: 1 plane full influence area longest split ===
	if constexpr (PLANE_FULL_PARALLEL_LONGEST || WOOD_SCENE) {
		// === Wood scene: 1 plane influence area parallel to x covering all the scene. 1 ray caster relative to the influence area. ===
		{
			PlaneInfluenceArea influenceAreaPlaneFullParallel{ Plane{{2.5,.9,0}, {-1,0,0}}, {1.5,0.9}, 10, 10000 };
			PlaneRayCaster rayCasterPlaneFullParallel{ influenceAreaPlaneFullParallel }; rayCasterPlaneFullParallel.generateRays(rng, 1000, true);
			Bvh bvhPlaneFullParallel{ bvhProperties, influenceAreaPlaneFullParallel, bvhStrategies::computeCostPah, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
			TopLevelOctree topLevel{ topLevelProperties, TopLevelOctree::OctreeProperties{ 4, false }, fallbackBvh, std::move(bvhPlaneFullParallel) };
			auto scene = TestScene{ string(RESULTS_DIRECTORY) + "WoodPlaneFullParallelLongest", woodTriangles, vector{&rayCasterPlaneFullParallel}, std::move(topLevel), topLevelAnalyzer };
			scene.buildAndTraverse();
		}

		// === Wood scene: 1 plane influence area 15 degrees covering all the scene. 1 ray caster relative to the influence area. ===
		{
			PlaneInfluenceArea influenceAreaPlaneFull15{ Plane{{2.5,.8,-.3}, {-.966,0,.259}}, {1.6,1}, 10, 10000 };
			PlaneRayCaster rayCasterPlaneFull15{ influenceAreaPlaneFull15 }; rayCasterPlaneFull15.generateRays(rng, 1000, true);
			Bvh bvhPlaneFull15{ bvhProperties, influenceAreaPlaneFull15, bvhStrategies::computeCostPah, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
			TopLevelOctree topLevel{ topLevelProperties, TopLevelOctree::OctreeProperties{ 4, false }, fallbackBvh, std::move(bvhPlaneFull15) };
			auto scene = TestScene{ string(RESULTS_DIRECTORY) + "WoodPlaneFull15Longest", woodTriangles, vector{&rayCasterPlaneFull15}, std::move(topLevel), topLevelAnalyzer };
			scene.buildAndTraverse();
		}

		// === Wood scene: 1 plane influence area 45 degrees covering all the scene. 1 ray caster relative to the influence area. ===
		{
			PlaneInfluenceArea influenceAreaPlaneFull45{ Plane{{2.5,.8,-2.5}, {-1,0,1}}, {2,1}, 10, 10000 };
			PlaneRayCaster rayCasterPlaneFull45{ influenceAreaPlaneFull45 }; rayCasterPlaneFull45.generateRays(rng, 1000, true);
			Bvh bvhPlaneFull45{ bvhProperties, influenceAreaPlaneFull45, bvhStrategies::computeCostPah, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
			TopLevelOctree topLevel{ topLevelProperties, TopLevelOctree::OctreeProperties{ 4, false }, fallbackBvh, std::move(bvhPlaneFull45) };
			auto scene = TestScene{ string(RESULTS_DIRECTORY) + "WoodPlaneFull45Longest", woodTriangles, vector{&rayCasterPlaneFull45}, std::move(topLevel), topLevelAnalyzer };
			scene.buildAndTraverse();
		}

		// === Wood scene: 1 plane influence area oblique covering all the scene. 1 ray caster relative to the influence area. ===
		{
			PlaneInfluenceArea influenceAreaPlaneFullOblique{ Plane{{2.5,1.5,-1}, {-1,-0.3,0.4}}, {1.8,1.3}, 20, 10000 };
			PlaneRayCaster rayCasterPlaneFullOblique{ influenceAreaPlaneFullOblique }; rayCasterPlaneFullOblique.generateRays(rng, 1000, true);
			Bvh bvhPlaneFullOblique{ bvhProperties, influenceAreaPlaneFullOblique, bvhStrategies::computeCostPah, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
			TopLevelOctree topLevel{ topLevelProperties, TopLevelOctree::OctreeProperties{ 4, false }, fallbackBvh, std::move(bvhPlaneFullOblique) };
			auto scene = TestScene{ string(RESULTS_DIRECTORY) + "WoodPlaneFullObliqueLongest", woodTriangles, vector{&rayCasterPlaneFullOblique}, std::move(topLevel), topLevelAnalyzer };
			scene.buildAndTraverse();
		}
	}

	// === Suzanne scene: 1 plane full influence area longest split ===
	if constexpr (PLANE_FULL_PARALLEL_LONGEST || SUZANNE_SCENE) {
		// === Suzanne scene: 1 plane influence area parallel to x covering all the scene. 1 ray caster relative to the influence area. ===
		{
			PlaneInfluenceArea influenceAreaPlaneFullParallel{ Plane{{0,0,2}, {0,0,-1}}, {.9,.9}, 10, 10000 };
			PlaneRayCaster rayCasterPlaneFullParallel{ influenceAreaPlaneFullParallel }; rayCasterPlaneFullParallel.generateRays(rng, 1000, true);
			Bvh bvhPlaneFullParallel{ bvhProperties, influenceAreaPlaneFullParallel, bvhStrategies::computeCostPah, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
			TopLevelOctree topLevel{ topLevelProperties, TopLevelOctree::OctreeProperties{ 4, false }, fallbackBvh, std::move(bvhPlaneFullParallel) };
			auto scene = TestScene{ string(RESULTS_DIRECTORY) + "SuzannePlaneFullParallelLongest", suzanneTriangles, vector{&rayCasterPlaneFullParallel}, std::move(topLevel), topLevelAnalyzer };
			scene.buildAndTraverse();
		}

		// === Suzanne scene: 1 plane influence area 15 degrees covering all the scene. 1 ray caster relative to the influence area. ===
		{
			PlaneInfluenceArea influenceAreaPlaneFull15{ Plane{{.5,0,2}, {-.259,0,-.966}}, {1,.9}, 10, 10000 };
			PlaneRayCaster rayCasterPlaneFull15{ influenceAreaPlaneFull15 }; rayCasterPlaneFull15.generateRays(rng, 1000, true);
			Bvh bvhPlaneFull15{ bvhProperties, influenceAreaPlaneFull15, bvhStrategies::computeCostPah, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
			TopLevelOctree topLevel{ topLevelProperties, TopLevelOctree::OctreeProperties{ 4, false }, fallbackBvh, std::move(bvhPlaneFull15) };
			auto scene = TestScene{ string(RESULTS_DIRECTORY) + "SuzannePlaneFull15Longest", suzanneTriangles, vector{&rayCasterPlaneFull15}, std::move(topLevel), topLevelAnalyzer };
			scene.buildAndTraverse();
		}

		// === Suzanne scene: 1 plane influence area 45 degrees covering all the scene. 1 ray caster relative to the influence area. ===
		{
			PlaneInfluenceArea influenceAreaPlaneFull45{ Plane{{2,0,2}, {-1,0,-1}}, {1.2,.9}, 10, 10000 };
			PlaneRayCaster rayCasterPlaneFull45{ influenceAreaPlaneFull45 }; rayCasterPlaneFull45.generateRays(rng, 1000, true);
			Bvh bvhPlaneFull45{ bvhProperties, influenceAreaPlaneFull45, bvhStrategies::computeCostPah, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
			TopLevelOctree topLevel{ topLevelProperties, TopLevelOctree::OctreeProperties{ 4, false }, fallbackBvh, std::move(bvhPlaneFull45) };
			auto scene = TestScene{ string(RESULTS_DIRECTORY) + "SuzannePlaneFull45Longest", suzanneTriangles, vector{&rayCasterPlaneFull45}, std::move(topLevel), topLevelAnalyzer };
			scene.buildAndTraverse();
		}

		// === Suzanne scene: 1 plane influence area oblique covering all the scene. 1 ray caster relative to the influence area. ===
		{
			PlaneInfluenceArea influenceAreaPlaneFullOblique{ Plane{{1,-.5,2}, {-.45,.3,-1}}, {.9,.9}, 10, 10000 };
			PlaneRayCaster rayCasterPlaneFullOblique{ influenceAreaPlaneFullOblique }; rayCasterPlaneFullOblique.generateRays(rng, 1000, true);
			Bvh bvhPlaneFullOblique{ bvhProperties, influenceAreaPlaneFullOblique, bvhStrategies::computeCostPah, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
			TopLevelOctree topLevel{ topLevelProperties, TopLevelOctree::OctreeProperties{ 4, false }, fallbackBvh, std::move(bvhPlaneFullOblique) };
			auto scene = TestScene{ string(RESULTS_DIRECTORY) + "SuzannePlaneFullObliqueLongest", suzanneTriangles, vector{&rayCasterPlaneFullOblique}, std::move(topLevel), topLevelAnalyzer };
			scene.buildAndTraverse();
		}
	}

	// === Random scene: 1 plane full influence area longest split ===
	if constexpr (PLANE_FULL_PARALLEL_LONGEST || RANDOM_SCENE) {
		// === Random scene: 1 plane influence area parallel to x covering all the scene. 1 ray caster relative to the influence area. ===
		{
			PlaneInfluenceArea influenceAreaPlaneFullParallel{ Plane{{5,5,-4}, {0,0,1}}, {5,5}, 20, 10000 };
			PlaneRayCaster rayCasterPlaneFullParallel{ influenceAreaPlaneFullParallel }; rayCasterPlaneFullParallel.generateRays(rng, 1000, true);
			Bvh bvhPlaneFullParallel{ bvhProperties, influenceAreaPlaneFullParallel, bvhStrategies::computeCostPah, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
			TopLevelOctree topLevel{ topLevelProperties, TopLevelOctree::OctreeProperties{ 4, false }, fallbackBvh, std::move(bvhPlaneFullParallel) };
			auto scene = TestScene{ string(RESULTS_DIRECTORY) + "RandomPlaneFullParallelLongest", randomTriangles, vector{&rayCasterPlaneFullParallel}, std::move(topLevel), topLevelAnalyzer };
			scene.buildAndTraverse();
		}

		// === Random scene: 1 plane influence area 15 degrees covering all the scene. 1 ray caster relative to the influence area. ===
		{
			PlaneInfluenceArea influenceAreaPlaneFull15{ Plane{{3.5,5,-4}, {.259,0,.966}}, {6.5,5}, 20, 10000 };
			PlaneRayCaster rayCasterPlaneFull15{ influenceAreaPlaneFull15 }; rayCasterPlaneFull15.generateRays(rng, 1000, true);
			Bvh bvhPlaneFull15{ bvhProperties, influenceAreaPlaneFull15, bvhStrategies::computeCostPah, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
			TopLevelOctree topLevel{ topLevelProperties, TopLevelOctree::OctreeProperties{ 4, false }, fallbackBvh, std::move(bvhPlaneFull15) };
			auto scene = TestScene{ string(RESULTS_DIRECTORY) + "RandomPlaneFull15Longest", randomTriangles, vector{&rayCasterPlaneFull15}, std::move(topLevel), topLevelAnalyzer };
			scene.buildAndTraverse();
		}

		// === Random scene: 1 plane influence area 45 degrees covering all the scene. 1 ray caster relative to the influence area. ===
		{
			PlaneInfluenceArea influenceAreaPlaneFull45{ Plane{{-3,5,-3}, {1,0,1}}, {7,5}, 20, 10000 };
			PlaneRayCaster rayCasterPlaneFull45{ influenceAreaPlaneFull45 }; rayCasterPlaneFull45.generateRays(rng, 1000, true);
			Bvh bvhPlaneFull45{ bvhProperties, influenceAreaPlaneFull45, bvhStrategies::computeCostPah, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
			TopLevelOctree topLevel{ topLevelProperties, TopLevelOctree::OctreeProperties{ 4, false }, fallbackBvh, std::move(bvhPlaneFull45) };
			auto scene = TestScene{ string(RESULTS_DIRECTORY) + "RandomPlaneFull45Longest", randomTriangles, vector{&rayCasterPlaneFull45}, std::move(topLevel), topLevelAnalyzer };
			scene.buildAndTraverse();
		}

		// === Random scene: 1 plane influence area oblique covering all the scene. 1 ray caster relative to the influence area. ===
		{
			PlaneInfluenceArea influenceAreaPlaneFullOblique{ Plane{{10,12,-3}, {-.55,-.9,1}}, {6.5,6.5}, 20, 10000 };
			PlaneRayCaster rayCasterPlaneFullOblique{ influenceAreaPlaneFullOblique }; rayCasterPlaneFullOblique.generateRays(rng, 1000, true);
			Bvh bvhPlaneFullOblique{ bvhProperties, influenceAreaPlaneFullOblique, bvhStrategies::computeCostPah, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
			TopLevelOctree topLevel{ topLevelProperties, TopLevelOctree::OctreeProperties{ 4, false }, fallbackBvh, std::move(bvhPlaneFullOblique) };
			auto scene = TestScene{ string(RESULTS_DIRECTORY) + "RandomPlaneFullObliqueLongest", randomTriangles, vector{&rayCasterPlaneFullOblique}, std::move(topLevel), topLevelAnalyzer };
			scene.buildAndTraverse();
		}
	}



	return 0;
}
