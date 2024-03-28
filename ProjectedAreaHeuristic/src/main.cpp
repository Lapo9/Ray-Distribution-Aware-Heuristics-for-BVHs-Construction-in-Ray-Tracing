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
#include "CsvExporter.h"

#include <iostream>
#include <memory>

using namespace std;
using namespace pah;
using namespace nlohmann;


int main() {
	//generate triangles
	mt19937 rng{ 1 };

	// select data to export to the csv file
	using AT = AnalysisType;
	CsvExporter csvTraversal{
		ACCESSOR("Estimated PAH cost",					AT::TOP_LEVEL,["bvhs"].at(0)["globalInfo"]["pahCost"]),
		ACCESSOR("Estimated SAH cost",					AT::TOP_LEVEL,["bvhs"].back()["globalInfo"]["sahCost"]),
		ACCESSOR("PAH cost with fallback",				AT::PAH,["cost"]["traversalCostAveragePerRay"]),
		ACCESSOR("PAH intersections with fallback",		AT::PAH,["total"]["intersectionTests"]["intersectionTestsAveragePerRay"]),
		ACCESSOR("PAH hit percentage",					AT::PAH,["total"]["hitMiss"]["hitsPercentage"]),
		ACCESSOR("PAH cost without fallback",			AT::PAH,["cost"]["traversalCostForBvhPerRay"].at(0).at(1)),
		ACCESSOR("SAH cost without fallback",			AT::PAH,["cost"]["traversalCostForBvhPerRay"].back().at(1)),
		ACCESSOR("PAH intersections without fallback",	AT::PAH,["fallback"]["intersectionTests"]["intersectionTestsNonFallbackAveragePerRay"]),
		ACCESSOR("SAH cost",							AT::FALLBACK,["cost"]["traversalCostAveragePerRay"]),
		ACCESSOR("SAH intersections",					AT::FALLBACK,["fallback"]["intersectionTests"]["intersectionTestsNonFallbackAveragePerRay"]),
		ACCESSOR("Overlapping %",						AT::TOP_LEVEL,["bvhs"].at(0)["globalInfo"]["siblingsOverlappingPercentage"]),
		ACCESSOR("Overlapping % culled",				AT::TOP_LEVEL,["bvhs"].at(0)["globalInfo"]["siblingsOverlappingPercentageCulled"]),
		ACCESSOR("Max level PAH",						AT::TOP_LEVEL,["bvhs"].at(0)["globalInfo"]["maxLevel"]),
		ACCESSOR("Max leaf cost",						AT::TOP_LEVEL,["bvhs"].at(0)["globalInfo"]["properties"]["maxLeafCost"]),
		ACCESSOR("Max leaf area",						AT::TOP_LEVEL,["bvhs"].at(0)["globalInfo"]["properties"]["maxLeafArea"]),
		ACCESSOR("Max leaf hit probability",			AT::TOP_LEVEL,["bvhs"].at(0)["globalInfo"]["properties"]["maxLeafHitProbability"]),
		ACCESSOR("Max triangles per leaf",				AT::TOP_LEVEL,["bvhs"].at(0)["globalInfo"]["properties"]["maxTrianglesPerLeaf"]),
		ACCESSOR("Max non fallback levels",				AT::TOP_LEVEL,["bvhs"].at(0)["globalInfo"]["properties"]["maxNonFallbackLevels"]),
		ACCESSOR("Split plane quality threshold",		AT::TOP_LEVEL,["bvhs"].at(0)["globalInfo"]["properties"]["splitPlaneQualityThreshold"]),
		ACCESSOR("Max children/father hit probability",	AT::TOP_LEVEL,["bvhs"].at(0)["globalInfo"]["properties"]["maxChildrenFatherHitProbabilityRatio"]),
		ACCESSOR("Choose split plane PAH average",		AT::TOP_LEVEL,["bvhs"].at(0)["totalTiming"]["chooseSplittingPlanesAverage"]),
		ACCESSOR("Choose split plane SAH average",		AT::TOP_LEVEL,["bvhs"].back()["totalTiming"]["chooseSplittingPlanesAverage"]),
		ACCESSOR("Compute cost PAH average",			AT::TOP_LEVEL,["bvhs"].at(0)["totalTiming"]["computeCostAverage"]),
		ACCESSOR("Compute cost SAH average",			AT::TOP_LEVEL,["bvhs"].back()["totalTiming"]["computeCostAverage"])
	};

	//generate triangles for different scenes
	distributions::UniformBoxDistribution mainDistribution3d{ 0,10, 0,10, 0,10 };
	distributions::UniformBoxDistribution otherDistribution3d{ -1,1, -1,1 , -1,1 };
	const vector random100Triangles = Triangle::generateRandom(100, rng, mainDistribution3d, otherDistribution3d);
	const vector random1000Triangles = Triangle::generateRandom(1000, rng, mainDistribution3d, otherDistribution3d);
	const vector woodTriangles = Triangle::fromObj("D:/Users/lapof/Documents/Development/ProjectedAreaHeuristic/ProjectedAreaHeuristic/exampleData/WoodScene.obj");
	const vector suzanneTriangles = Triangle::fromObj("D:/Users/lapof/Documents/Development/ProjectedAreaHeuristic/ProjectedAreaHeuristic/exampleData/MonkeyModel.obj");
	const vector cottageTriangles = Triangle::fromObj("D:/Users/lapof/Documents/Development/ProjectedAreaHeuristic/ProjectedAreaHeuristic/exampleData/Cottage.obj");
	const vector cottageWallsTriangles = Triangle::fromObj("D:/Users/lapof/Documents/Development/ProjectedAreaHeuristic/ProjectedAreaHeuristic/exampleData/CottageWalls.obj");
	const vector crowdTriangles = Triangle::fromObj("D:/Users/lapof/Documents/Development/ProjectedAreaHeuristic/ProjectedAreaHeuristic/exampleData/Crowd.obj");
	const vector sponzaTriangles = Triangle::fromObj("D:/Users/lapof/Documents/Development/ProjectedAreaHeuristic/ProjectedAreaHeuristic/exampleData/Cube.obj");

	//Top level analyzer used by most scenes
	TopLevelOctreeAnalyzer topLevelAnalyzer{
		MAKE_ACTIONS_PAIR(core),
		MAKE_ACTIONS_PAIR(sah),
		MAKE_ACTIONS_PAIR(pah),
		MAKE_ACTIONS_PAIR(levelCount),
		MAKE_ACTIONS_PAIR(triangles),
		MAKE_ACTIONS_PAIR(influenceArea),
		std::pair{ std::function{ analyzerActions::perNode::siblingsOverlapping<4> }, std::function{ analyzerActions::finals::siblingsOverlapping }},
		MAKE_ACTIONS_PAIR(timeMeasurement)
	};

	//properties for most BVHs
	Bvh::Properties bvhProperties{
	.maxLeafCost = 0.0f,
	.maxLeafArea = 0.0f,
	.maxLeafHitProbability = 0.0f,
	.maxTrianglesPerLeaf = 2,
	.maxLevels = 100,
	.bins = 40,
	.maxNonFallbackLevels = 100,
	.splitPlaneQualityThreshold = 0.9f,
	.acceptableChildrenFatherHitProbabilityRatio = 1.3f,
	.excellentChildrenFatherHitProbabilityRatio = 0.9f
	};


	TopLevelOctree::OctreeProperties octreeProperties{
		.maxLevel = 4,
		.conservativeApproach = false
	};

	//fallback BVH used by most top level structures
	Bvh fallbackBvh{ bvhProperties, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesLongest<0.f>, bvhStrategies::shouldStopThresholdOrLevel, "fallback" };
	fallbackBvh.setFallbackComputeCostStrategy(bvhStrategies::computeCostSah);


	// Filter test scenes to run
#define and_or &&
	constexpr bool ALL = false;

	constexpr bool PLANE_FULL = ALL || true;
	constexpr bool PLANE_FULL_LONGEST = ALL || true;
	constexpr bool POINT_FULL = ALL || true;
	constexpr bool POINT_FULL_LONGEST = ALL || false;

	constexpr bool WOOD_SCENE = ALL || false;
	constexpr bool SUZANNE_SCENE = ALL || false;
	constexpr bool COTTAGE_SCENE = ALL || false;
	constexpr bool COTTAGE_WALLS_SCENE = ALL || false;
	constexpr bool CROWD_SCENE = ALL || false;
	constexpr bool RANDOM100_SCENE = ALL || false;
	constexpr bool RANDOM1000_SCENE = ALL || false;
	constexpr bool SPONZA_SCENE = ALL || true;

	constexpr string_view RESULTS_DIRECTORY = "D:/Users/lapof/Documents/Development/ProjectedAreaHeuristic/Results/";

	// PLANE FULL PARALLEL
	bvhProperties.splitPlaneQualityThreshold = 0.4f;
	{
		// === Wood scene: 1 plane full influence area ===
		if constexpr (PLANE_FULL and_or WOOD_SCENE) {
			// === Wood scene: 1 plane influence area parallel to x covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlaneFullParallel{ Plane{{2.5,.9,0}, {-1,0,0}, 1.5,0.9}, 10, 10000 };
				PlaneRayCaster rayCasterPlaneFullParallel{ influenceAreaPlaneFullParallel }; rayCasterPlaneFullParallel.generateRays(rng, 1000, true);
				Bvh bvhPlaneFullParallel{ bvhProperties, influenceAreaPlaneFullParallel, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlaneFullParallel) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "WoodPlaneFullParallel", woodTriangles, vector{&rayCasterPlaneFullParallel}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("WoodPlaneFullParallel", scene.buildAndTraverse());
			}

			// === Wood scene: 1 plane influence area 15 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlaneFull15{ Plane{{2.5,.8,-.3}, {-.966,0,.259}, 1.6,1}, 10, 10000 };
				PlaneRayCaster rayCasterPlaneFull15{ influenceAreaPlaneFull15 }; rayCasterPlaneFull15.generateRays(rng, 1000, true);
				Bvh bvhPlaneFull15{ bvhProperties, influenceAreaPlaneFull15, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlaneFull15) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "WoodPlaneFull15", woodTriangles, vector{&rayCasterPlaneFull15}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("WoodPlaneFull15", scene.buildAndTraverse());
			}

			// === Wood scene: 1 plane influence area 45 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlaneFull45{ Plane{{2.5,.8,-3}, {-1,0,1}, 1.5,1}, 10, 10000 };
				PlaneRayCaster rayCasterPlaneFull45{ influenceAreaPlaneFull45 }; rayCasterPlaneFull45.generateRays(rng, 1000, true);
				Bvh bvhPlaneFull45{ bvhProperties, influenceAreaPlaneFull45, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlaneFull45) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "WoodPlaneFull45", woodTriangles, vector{&rayCasterPlaneFull45}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("WoodPlaneFull45", scene.buildAndTraverse());
			}

			// === Wood scene: 1 plane influence area oblique covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlaneFullOblique{ Plane{{2.5,1.5,-1}, {-1,-0.3,0.4}, 1.8,1.3}, 20, 10000 };
				PlaneRayCaster rayCasterPlaneFullOblique{ influenceAreaPlaneFullOblique }; rayCasterPlaneFullOblique.generateRays(rng, 1000, true);
				Bvh bvhPlaneFullOblique{ bvhProperties, influenceAreaPlaneFullOblique, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlaneFullOblique) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "WoodPlaneFullOblique", woodTriangles, vector{&rayCasterPlaneFullOblique}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("WoodPlaneFullOblique", scene.buildAndTraverse());
			}
		}

		// === Suzanne scene: 1 plane full influence area ===
		if constexpr (PLANE_FULL and_or SUZANNE_SCENE) {
			// === Suzanne scene: 1 plane influence area parallel to x covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlaneFullParallel{ Plane{{0,0,2}, {0,0,-1}, .9,.9}, 10, 10000 };
				PlaneRayCaster rayCasterPlaneFullParallel{ influenceAreaPlaneFullParallel }; rayCasterPlaneFullParallel.generateRays(rng, 1000, true);
				Bvh bvhPlaneFullParallel{ bvhProperties, influenceAreaPlaneFullParallel, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlaneFullParallel) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "SuzannePlaneFullParallel", suzanneTriangles, vector{&rayCasterPlaneFullParallel}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("SuzannePlaneFullParallel", scene.buildAndTraverse());
			}

			// === Suzanne scene: 1 plane influence area 15 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlaneFull15{ Plane{{.5,0,2}, {-.259,0,-.966}, 1,.9}, 10, 10000 };
				PlaneRayCaster rayCasterPlaneFull15{ influenceAreaPlaneFull15 }; rayCasterPlaneFull15.generateRays(rng, 1000, true);
				Bvh bvhPlaneFull15{ bvhProperties, influenceAreaPlaneFull15, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlaneFull15) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "SuzannePlaneFull15", suzanneTriangles, vector{&rayCasterPlaneFull15}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("SuzannePlaneFull15", scene.buildAndTraverse());
			}

			// === Suzanne scene: 1 plane influence area 45 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlaneFull45{ Plane{{2,0,2}, {-1,0,-1}, 1.2,.9}, 10, 10000 };
				PlaneRayCaster rayCasterPlaneFull45{ influenceAreaPlaneFull45 }; rayCasterPlaneFull45.generateRays(rng, 1000, true);
				Bvh bvhPlaneFull45{ bvhProperties, influenceAreaPlaneFull45, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlaneFull45) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "SuzannePlaneFull45", suzanneTriangles, vector{&rayCasterPlaneFull45}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("SuzannePlaneFull45", scene.buildAndTraverse());
			}

			// === Suzanne scene: 1 plane influence area oblique covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlaneFullOblique{ Plane{{1,-.5,2}, {-.45,.3,-1}, 2,1.5}, 10, 10000 };
				PlaneRayCaster rayCasterPlaneFullOblique{ influenceAreaPlaneFullOblique }; rayCasterPlaneFullOblique.generateRays(rng, 1000, true);
				Bvh bvhPlaneFullOblique{ bvhProperties, influenceAreaPlaneFullOblique, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlaneFullOblique) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "SuzannePlaneFullOblique", suzanneTriangles, vector{&rayCasterPlaneFullOblique}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("SuzannePlaneFullOblique", scene.buildAndTraverse());
			}
		}

		// === Cottage scene: 1 plane full influence area ===
		if constexpr (PLANE_FULL and_or COTTAGE_SCENE) {
			// === Cottage scene: 1 plane influence area parallel to x covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlaneFullParallel{ Plane{{10,2.8,0}, {-1,0,0}, 6,2.6}, 35, 10000 };
				PlaneRayCaster rayCasterPlaneFullParallel{ influenceAreaPlaneFullParallel }; rayCasterPlaneFullParallel.generateRays(rng, 1000, true);
				Bvh bvhPlaneFullParallel{ bvhProperties, influenceAreaPlaneFullParallel, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlaneFullParallel) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "CottagePlaneFullParallel", cottageTriangles, vector{&rayCasterPlaneFullParallel}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("CottagePlaneFullParallel", scene.buildAndTraverse());
			}

			// === Cottage scene: 1 plane influence area 15 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlaneFull15{ Plane{{10,2.9,-3}, {-.966,0,.259}, 6,2.9}, 35, 10000 };
				PlaneRayCaster rayCasterPlaneFull15{ influenceAreaPlaneFull15 }; rayCasterPlaneFull15.generateRays(rng, 1000, true);
				Bvh bvhPlaneFull15{ bvhProperties, influenceAreaPlaneFull15, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlaneFull15) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "CottagePlaneFull15", cottageTriangles, vector{&rayCasterPlaneFull15}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("CottagePlaneFull15", scene.buildAndTraverse());
			}

			// === Cottage scene: 1 plane influence area 45 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlaneFull45{ Plane{{7.5,2.9,-7.5}, {-1,0,1}, 8,2.9}, 35, 10000 };
				PlaneRayCaster rayCasterPlaneFull45{ influenceAreaPlaneFull45 }; rayCasterPlaneFull45.generateRays(rng, 1000, true);
				Bvh bvhPlaneFull45{ bvhProperties, influenceAreaPlaneFull45, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlaneFull45) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "CottagePlaneFull45", cottageTriangles, vector{&rayCasterPlaneFull45}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("CottagePlaneFull45", scene.buildAndTraverse());
			}

			// === Cottage scene: 1 plane influence area oblique covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlaneFullOblique{ Plane{{10,5,-8}, {-1,-.3,.9}, 7,3.3}, 35, 10000 };
				PlaneRayCaster rayCasterPlaneFullOblique{ influenceAreaPlaneFullOblique }; rayCasterPlaneFullOblique.generateRays(rng, 1000, true);
				Bvh bvhPlaneFullOblique{ bvhProperties, influenceAreaPlaneFullOblique, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlaneFullOblique) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "CottagePlaneFullOblique", cottageTriangles, vector{&rayCasterPlaneFullOblique}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("CottagePlaneFullOblique", scene.buildAndTraverse());
			}
		}

		// === CottageWalls scene: 1 plane full influence area ===
		if constexpr (PLANE_FULL and_or COTTAGE_WALLS_SCENE) {
			// === CottageWalls scene: 1 plane influence area parallel to x covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlaneFullParallel{ Plane{{10,2.8,0}, {-1,0,0}, 6,2.6}, 35, 10000 };
				PlaneRayCaster rayCasterPlaneFullParallel{ influenceAreaPlaneFullParallel }; rayCasterPlaneFullParallel.generateRays(rng, 1000, true);
				Bvh bvhPlaneFullParallel{ bvhProperties, influenceAreaPlaneFullParallel, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlaneFullParallel) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "CottageWallsPlaneFullParallel", cottageWallsTriangles, vector{&rayCasterPlaneFullParallel}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("CottageWallsPlaneFullParallel", scene.buildAndTraverse());
			}

			// === CottageWalls scene: 1 plane influence area 15 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlaneFull15{ Plane{{10,2.9,-3}, {-.966,0,.259}, 6,2.9}, 35, 10000 };
				PlaneRayCaster rayCasterPlaneFull15{ influenceAreaPlaneFull15 }; rayCasterPlaneFull15.generateRays(rng, 1000, true);
				Bvh bvhPlaneFull15{ bvhProperties, influenceAreaPlaneFull15, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlaneFull15) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "CottageWallsPlaneFull15", cottageWallsTriangles, vector{&rayCasterPlaneFull15}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("CottageWallsPlaneFull15", scene.buildAndTraverse());
			}

			// === CottageWalls scene: 1 plane influence area 45 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlaneFull45{ Plane{{7.5,2.9,-7.5}, {-1,0,1}, 8,2.9}, 35, 10000 };
				PlaneRayCaster rayCasterPlaneFull45{ influenceAreaPlaneFull45 }; rayCasterPlaneFull45.generateRays(rng, 1000, true);
				Bvh bvhPlaneFull45{ bvhProperties, influenceAreaPlaneFull45, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlaneFull45) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "CottageWallsPlaneFull45", cottageWallsTriangles, vector{&rayCasterPlaneFull45}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("CottageWallsPlaneFull45", scene.buildAndTraverse());
			}

			// === CottageWalls scene: 1 plane influence area oblique covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlaneFullOblique{ Plane{{10,5,-8}, {-1,-.3,.9}, 7,3.3}, 35, 10000 };
				PlaneRayCaster rayCasterPlaneFullOblique{ influenceAreaPlaneFullOblique }; rayCasterPlaneFullOblique.generateRays(rng, 1000, true);
				Bvh bvhPlaneFullOblique{ bvhProperties, influenceAreaPlaneFullOblique, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlaneFullOblique) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "CottageWallsPlaneFullOblique", cottageWallsTriangles, vector{&rayCasterPlaneFullOblique}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("CottageWallsPlaneFullOblique", scene.buildAndTraverse());
			}
		}

		// === Crowd scene: 1 plane full influence area ===
		if constexpr (PLANE_FULL and_or CROWD_SCENE) {
			// === Crowd scene: 1 plane influence area parallel to x covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlaneFullParallel{ Plane{{2,2.2,-8}, {0,0,1}, 7,1.8}, 30, 10000 };
				PlaneRayCaster rayCasterPlaneFullParallel{ influenceAreaPlaneFullParallel }; rayCasterPlaneFullParallel.generateRays(rng, 1000, true);
				Bvh bvhPlaneFullParallel{ bvhProperties, influenceAreaPlaneFullParallel, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlaneFullParallel) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "CrowdPlaneFullParallel", crowdTriangles, vector{&rayCasterPlaneFullParallel}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("CrowdPlaneFullParallel", scene.buildAndTraverse());
			}

			// === Crowd scene: 1 plane influence area 15 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlaneFull15{ Plane{{-2.5,2.2,-8}, {.259,0,.966}, 7.5,1.8}, 30, 10000 };
				PlaneRayCaster rayCasterPlaneFull15{ influenceAreaPlaneFull15 }; rayCasterPlaneFull15.generateRays(rng, 1000, true);
				Bvh bvhPlaneFull15{ bvhProperties, influenceAreaPlaneFull15, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlaneFull15) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "CrowdPlaneFull15", crowdTriangles, vector{&rayCasterPlaneFull15}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("CrowdPlaneFull15", scene.buildAndTraverse());
			}

			// === Crowd scene: 1 plane influence area 45 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlaneFull45{ Plane{{-8,2.2,-8}, {1,0,1}, 9,1.8}, 30, 10000 };
				PlaneRayCaster rayCasterPlaneFull45{ influenceAreaPlaneFull45 }; rayCasterPlaneFull45.generateRays(rng, 1000, true);
				Bvh bvhPlaneFull45{ bvhProperties, influenceAreaPlaneFull45, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlaneFull45) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "CrowdPlaneFull45", crowdTriangles, vector{&rayCasterPlaneFull45}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("CrowdPlaneFull45", scene.buildAndTraverse());
			}

			// === Crowd scene: 1 plane influence area oblique covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlaneFullOblique{ Plane{{-6,7,-8}, {.6,-.4,1}, 7.5,3.4}, 35, 10000 };
				PlaneRayCaster rayCasterPlaneFullOblique{ influenceAreaPlaneFullOblique }; rayCasterPlaneFullOblique.generateRays(rng, 1000, true);
				Bvh bvhPlaneFullOblique{ bvhProperties, influenceAreaPlaneFullOblique, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlaneFullOblique) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "CrowdPlaneFullOblique", crowdTriangles, vector{&rayCasterPlaneFullOblique}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("CrowdPlaneFullOblique", scene.buildAndTraverse());
			}
		}

		// === Random100 scene: 1 plane full influence area ===
		if constexpr (PLANE_FULL and_or RANDOM100_SCENE) {
			// === Random100 scene: 1 plane influence area parallel to x covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlaneFullParallel{ Plane{{5,5,-4}, {0,0,1}, 5,5}, 20, 10000 };
				PlaneRayCaster rayCasterPlaneFullParallel{ influenceAreaPlaneFullParallel }; rayCasterPlaneFullParallel.generateRays(rng, 1000, true);
				Bvh bvhPlaneFullParallel{ bvhProperties, influenceAreaPlaneFullParallel, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlaneFullParallel) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Random100PlaneFullParallel", random100Triangles, vector{&rayCasterPlaneFullParallel}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Random100PlaneFullParallel", scene.buildAndTraverse());
			}

			// === Random100 scene: 1 plane influence area 15 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlaneFull15{ Plane{{3.5,5,-4}, {.259,0,.966}, 6.5,5}, 20, 10000 };
				PlaneRayCaster rayCasterPlaneFull15{ influenceAreaPlaneFull15 }; rayCasterPlaneFull15.generateRays(rng, 1000, true);
				Bvh bvhPlaneFull15{ bvhProperties, influenceAreaPlaneFull15, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlaneFull15) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Random100PlaneFull15", random100Triangles, vector{&rayCasterPlaneFull15}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Random100PlaneFull15", scene.buildAndTraverse());
			}

			// === Random100 scene: 1 plane influence area 45 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlaneFull45{ Plane{{-3,5,-3}, {1,0,1}, 7,5}, 20, 10000 };
				PlaneRayCaster rayCasterPlaneFull45{ influenceAreaPlaneFull45 }; rayCasterPlaneFull45.generateRays(rng, 1000, true);
				Bvh bvhPlaneFull45{ bvhProperties, influenceAreaPlaneFull45, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlaneFull45) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Random100PlaneFull45", random100Triangles, vector{&rayCasterPlaneFull45}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Random100PlaneFull45", scene.buildAndTraverse());
			}

			// === Random100 scene: 1 plane influence area oblique covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlaneFullOblique{ Plane{{10,12,-3}, {-.55,-.9,1}, 6.5,6.5}, 20, 10000 };
				PlaneRayCaster rayCasterPlaneFullOblique{ influenceAreaPlaneFullOblique }; rayCasterPlaneFullOblique.generateRays(rng, 1000, true);
				Bvh bvhPlaneFullOblique{ bvhProperties, influenceAreaPlaneFullOblique, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlaneFullOblique) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Random100PlaneFullOblique", random100Triangles, vector{&rayCasterPlaneFullOblique}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Random100PlaneFullOblique", scene.buildAndTraverse());
			}
		}

		// === Random1000 scene: 1 plane full influence area ===
		if constexpr (PLANE_FULL and_or RANDOM1000_SCENE) {
			// === Random1000 scene: 1 plane influence area parallel to x covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlaneFullParallel{ Plane{{5,5,-4}, {0,0,1}, 5,5}, 20, 10000 };
				PlaneRayCaster rayCasterPlaneFullParallel{ influenceAreaPlaneFullParallel }; rayCasterPlaneFullParallel.generateRays(rng, 1000, true);
				Bvh bvhPlaneFullParallel{ bvhProperties, influenceAreaPlaneFullParallel, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlaneFullParallel) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Random1000PlaneFullParallel", random1000Triangles, vector{&rayCasterPlaneFullParallel}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Random1000PlaneFullParallel", scene.buildAndTraverse());
			}

			// === Random1000 scene: 1 plane influence area 15 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlaneFull15{ Plane{{3.5,5,-4}, {.259,0,.966}, 6.5,5}, 20, 10000 };
				PlaneRayCaster rayCasterPlaneFull15{ influenceAreaPlaneFull15 }; rayCasterPlaneFull15.generateRays(rng, 1000, true);
				Bvh bvhPlaneFull15{ bvhProperties, influenceAreaPlaneFull15, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlaneFull15) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Random1000PlaneFull15", random1000Triangles, vector{&rayCasterPlaneFull15}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Random1000PlaneFull15", scene.buildAndTraverse());
			}

			// === Random1000 scene: 1 plane influence area 45 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlaneFull45{ Plane{{-3,5,-3}, {1,0,1}, 7,5}, 20, 10000 };
				PlaneRayCaster rayCasterPlaneFull45{ influenceAreaPlaneFull45 }; rayCasterPlaneFull45.generateRays(rng, 1000, true);
				Bvh bvhPlaneFull45{ bvhProperties, influenceAreaPlaneFull45, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlaneFull45) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Random1000PlaneFull45", random1000Triangles, vector{&rayCasterPlaneFull45}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Random1000PlaneFull45", scene.buildAndTraverse());
			}

			// === Random1000 scene: 1 plane influence area oblique covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlaneFullOblique{ Plane{{10,12,-3}, {-.55,-.9,1}, 6.5,6.5}, 20, 10000 };
				PlaneRayCaster rayCasterPlaneFullOblique{ influenceAreaPlaneFullOblique }; rayCasterPlaneFullOblique.generateRays(rng, 1000, true);
				Bvh bvhPlaneFullOblique{ bvhProperties, influenceAreaPlaneFullOblique, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlaneFullOblique) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Random1000PlaneFullOblique", random1000Triangles, vector{&rayCasterPlaneFullOblique}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Random1000PlaneFullOblique", scene.buildAndTraverse());
			}
		}

		// === Sponza scene: 1 plane full influence area ===
		if constexpr (PLANE_FULL and_or SPONZA_SCENE) {
			// === Sponza scene: 1 plane influence area parallel to x covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlaneFullParallel{ Plane{{-9,11,-1}, {1,0,0}, 16, 11}, 50, 10000 };
				PlaneRayCaster rayCasterPlaneFullParallel{ influenceAreaPlaneFullParallel }; rayCasterPlaneFullParallel.generateRays(rng, 1000, true);
				Bvh bvhPlaneFullParallel{ bvhProperties, influenceAreaPlaneFullParallel, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlaneFullParallel) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "SponzaPlaneFullParallel", sponzaTriangles, vector{&rayCasterPlaneFullParallel}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("SponzaPlaneFullParallel", scene.buildAndTraverse());
			}

			
			// === Sponza scene: 1 plane influence area 15 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlaneFull15{ Plane{{-12,11,0.5}, {.966,0,-.259}, 16,11}, 60, 10000 };
				PlaneRayCaster rayCasterPlaneFull15{ influenceAreaPlaneFull15 }; rayCasterPlaneFull15.generateRays(rng, 1000, true);
				Bvh bvhPlaneFull15{ bvhProperties, influenceAreaPlaneFull15, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlaneFull15) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "SponzaPlaneFull15", sponzaTriangles, vector{&rayCasterPlaneFull15}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("SponzaPlaneFull15", scene.buildAndTraverse());
			}

			// === Sponza scene: 1 plane influence area 45 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlaneFull45{ Plane{{-10,11,18}, {1,0,-1}, 25,11}, 70, 10000 };
				PlaneRayCaster rayCasterPlaneFull45{ influenceAreaPlaneFull45 }; rayCasterPlaneFull45.generateRays(rng, 1000, true);
				Bvh bvhPlaneFull45{ bvhProperties, influenceAreaPlaneFull45, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlaneFull45) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "SponzaPlaneFull45", sponzaTriangles, vector{&rayCasterPlaneFull45}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("SponzaPlaneFull45", scene.buildAndTraverse());
			}

			// === Sponza scene: 1 plane influence area oblique covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlaneFullOblique{ Plane{{-12,10,7.2}, {.84,0.26,-.48}, 16,11}, 70, 10000 };
				PlaneRayCaster rayCasterPlaneFullOblique{ influenceAreaPlaneFullOblique }; rayCasterPlaneFullOblique.generateRays(rng, 1000, true);
				Bvh bvhPlaneFullOblique{ bvhProperties, influenceAreaPlaneFullOblique, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlaneFullOblique) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "SponzaPlaneFullOblique", sponzaTriangles, vector{&rayCasterPlaneFullOblique}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("SponzaPlaneFullOblique", scene.buildAndTraverse());
			}
			
		}
	};

	// PLANE FULL PARALLEL LONGEST
	bvhProperties.splitPlaneQualityThreshold = 0.9f;
	{
		// === Wood scene: 1 plane full influence area longest split ===
		if constexpr (PLANE_FULL_LONGEST and_or WOOD_SCENE) {
			// === Wood scene: 1 plane influence area parallel to x covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlaneFullParallel{ Plane{{2.5,.9,0}, {-1,0,0}, 1.5,0.9}, 10, 10000 };
				PlaneRayCaster rayCasterPlaneFullParallel{ influenceAreaPlaneFullParallel }; rayCasterPlaneFullParallel.generateRays(rng, 1000, true);
				Bvh bvhPlaneFullParallel{ bvhProperties, influenceAreaPlaneFullParallel, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlaneFullParallel) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "WoodPlaneFullParallelLongest", woodTriangles, vector{&rayCasterPlaneFullParallel}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("WoodPlaneFullParallelLongest", scene.buildAndTraverse());
			}

			// === Wood scene: 1 plane influence area 15 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlaneFull15{ Plane{{2.5,.8,-.3}, {-.966,0,.259}, 1.6,1}, 10, 10000 };
				PlaneRayCaster rayCasterPlaneFull15{ influenceAreaPlaneFull15 }; rayCasterPlaneFull15.generateRays(rng, 1000, true);
				Bvh bvhPlaneFull15{ bvhProperties, influenceAreaPlaneFull15, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlaneFull15) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "WoodPlaneFull15Longest", woodTriangles, vector{&rayCasterPlaneFull15}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("WoodPlaneFull15Longest", scene.buildAndTraverse());
			}

			// === Wood scene: 1 plane influence area 45 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlaneFull45{ Plane{{2.5,.8,-3}, {-1,0,1}, 1.5,1}, 10, 10000 };
				PlaneRayCaster rayCasterPlaneFull45{ influenceAreaPlaneFull45 }; rayCasterPlaneFull45.generateRays(rng, 1000, true);
				Bvh bvhPlaneFull45{ bvhProperties, influenceAreaPlaneFull45, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlaneFull45) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "WoodPlaneFull45Longest", woodTriangles, vector{&rayCasterPlaneFull45}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("WoodPlaneFull45Longest", scene.buildAndTraverse());
			}

			// === Wood scene: 1 plane influence area oblique covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlaneFullOblique{ Plane{{2.5,1.5,-1}, {-1,-0.3,0.4}, 1.8,1.3}, 20, 10000 };
				PlaneRayCaster rayCasterPlaneFullOblique{ influenceAreaPlaneFullOblique }; rayCasterPlaneFullOblique.generateRays(rng, 1000, true);
				Bvh bvhPlaneFullOblique{ bvhProperties, influenceAreaPlaneFullOblique, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlaneFullOblique) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "WoodPlaneFullObliqueLongest", woodTriangles, vector{&rayCasterPlaneFullOblique}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("WoodPlaneFullObliqueLongest", scene.buildAndTraverse());
			}
		}

		// === Suzanne scene: 1 plane full influence area longest split ===
		if constexpr (PLANE_FULL_LONGEST and_or SUZANNE_SCENE) {
			// === Suzanne scene: 1 plane influence area parallel to x covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlaneFullParallel{ Plane{{0,0,2}, {0,0,-1}, .9,.9}, 10, 10000 };
				PlaneRayCaster rayCasterPlaneFullParallel{ influenceAreaPlaneFullParallel }; rayCasterPlaneFullParallel.generateRays(rng, 1000, true);
				Bvh bvhPlaneFullParallel{ bvhProperties, influenceAreaPlaneFullParallel, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlaneFullParallel) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "SuzannePlaneFullParallelLongest", suzanneTriangles, vector{&rayCasterPlaneFullParallel}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("SuzannePlaneFullParallelLongest", scene.buildAndTraverse());
			}

			// === Suzanne scene: 1 plane influence area 15 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlaneFull15{ Plane{{.5,0,2}, {-.259,0,-.966}, 1,.9}, 10, 10000 };
				PlaneRayCaster rayCasterPlaneFull15{ influenceAreaPlaneFull15 }; rayCasterPlaneFull15.generateRays(rng, 1000, true);
				Bvh bvhPlaneFull15{ bvhProperties, influenceAreaPlaneFull15, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlaneFull15) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "SuzannePlaneFull15Longest", suzanneTriangles, vector{&rayCasterPlaneFull15}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("SuzannePlaneFull15Longest", scene.buildAndTraverse());
			}

			// === Suzanne scene: 1 plane influence area 45 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlaneFull45{ Plane{{2,0,2}, {-1,0,-1}, 1.2,.9}, 10, 10000 };
				PlaneRayCaster rayCasterPlaneFull45{ influenceAreaPlaneFull45 }; rayCasterPlaneFull45.generateRays(rng, 1000, true);
				Bvh bvhPlaneFull45{ bvhProperties, influenceAreaPlaneFull45, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlaneFull45) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "SuzannePlaneFull45Longest", suzanneTriangles, vector{&rayCasterPlaneFull45}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("SuzannePlaneFull45Longest", scene.buildAndTraverse());
			}

			// === Suzanne scene: 1 plane influence area oblique covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlaneFullOblique{ Plane{{1,-.5,2}, {-.45,.3,-1}, .9,.9}, 10, 10000 };
				PlaneRayCaster rayCasterPlaneFullOblique{ influenceAreaPlaneFullOblique }; rayCasterPlaneFullOblique.generateRays(rng, 1000, true);
				Bvh bvhPlaneFullOblique{ bvhProperties, influenceAreaPlaneFullOblique, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlaneFullOblique) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "SuzannePlaneFullObliqueLongest", suzanneTriangles, vector{&rayCasterPlaneFullOblique}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("SuzannePlaneFullObliqueLongest", scene.buildAndTraverse());
			}
		}

		// === Cottage scene: 1 plane full influence area longest split ===
		if constexpr (PLANE_FULL_LONGEST and_or COTTAGE_SCENE) {
			// === Cottage scene: 1 plane influence area parallel to x covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlaneFullParallel{ Plane{{10,2.8,0}, {-1,0,0}, 6,2.6}, 35, 10000 };
				PlaneRayCaster rayCasterPlaneFullParallel{ influenceAreaPlaneFullParallel }; rayCasterPlaneFullParallel.generateRays(rng, 1000, true);
				Bvh bvhPlaneFullParallel{ bvhProperties, influenceAreaPlaneFullParallel, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlaneFullParallel) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "CottagePlaneFullParallelLongest", cottageTriangles, vector{&rayCasterPlaneFullParallel}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("CottagePlaneFullParallelLongest", scene.buildAndTraverse());
			}

			// === Cottage scene: 1 plane influence area 15 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlaneFull15{ Plane{{10,2.9,-3}, {-.966,0,.259}, 6,2.9}, 35, 10000 };
				PlaneRayCaster rayCasterPlaneFull15{ influenceAreaPlaneFull15 }; rayCasterPlaneFull15.generateRays(rng, 1000, true);
				Bvh bvhPlaneFull15{ bvhProperties, influenceAreaPlaneFull15, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlaneFull15) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "CottagePlaneFull15Longest", cottageTriangles, vector{&rayCasterPlaneFull15}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("CottagePlaneFull15Longest", scene.buildAndTraverse());
			}

			// === Cottage scene: 1 plane influence area 45 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlaneFull45{ Plane{{7.5,2.9,-7.5}, {-1,0,1}, 8,2.9}, 35, 10000 };
				PlaneRayCaster rayCasterPlaneFull45{ influenceAreaPlaneFull45 }; rayCasterPlaneFull45.generateRays(rng, 1000, true);
				Bvh bvhPlaneFull45{ bvhProperties, influenceAreaPlaneFull45, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlaneFull45) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "CottagePlaneFull45Longest", cottageTriangles, vector{&rayCasterPlaneFull45}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("CottagePlaneFull45Longest", scene.buildAndTraverse());
			}

			// === Cottage scene: 1 plane influence area oblique covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlaneFullOblique{ Plane{{10,5,-8}, {-1,-.3,.9}, 7,3.3}, 35, 10000 };
				PlaneRayCaster rayCasterPlaneFullOblique{ influenceAreaPlaneFullOblique }; rayCasterPlaneFullOblique.generateRays(rng, 1000, true);
				Bvh bvhPlaneFullOblique{ bvhProperties, influenceAreaPlaneFullOblique, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlaneFullOblique) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "CottagePlaneFullObliqueLongest", cottageTriangles, vector{&rayCasterPlaneFullOblique}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("CottagePlaneFullObliqueLongest", scene.buildAndTraverse());
			}
		}

		// === CottageWalls scene: 1 plane full influence area longest split ===
		if constexpr (PLANE_FULL_LONGEST and_or COTTAGE_WALLS_SCENE) {
			// === CottageWalls scene: 1 plane influence area parallel to x covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlaneFullParallel{ Plane{{10,2.8,0}, {-1,0,0}, 6,2.6}, 35, 10000 };
				PlaneRayCaster rayCasterPlaneFullParallel{ influenceAreaPlaneFullParallel }; rayCasterPlaneFullParallel.generateRays(rng, 1000, true);
				Bvh bvhPlaneFullParallel{ bvhProperties, influenceAreaPlaneFullParallel, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlaneFullParallel) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "CottageWallsPlaneFullParallelLongest", cottageWallsTriangles, vector{&rayCasterPlaneFullParallel}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("CottageWallsPlaneFullParallelLongest", scene.buildAndTraverse());
			}

			// === CottageWalls scene: 1 plane influence area 15 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlaneFull15{ Plane{{10,2.9,-3}, {-.966,0,.259}, 6,2.9}, 35, 10000 };
				PlaneRayCaster rayCasterPlaneFull15{ influenceAreaPlaneFull15 }; rayCasterPlaneFull15.generateRays(rng, 1000, true);
				Bvh bvhPlaneFull15{ bvhProperties, influenceAreaPlaneFull15, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlaneFull15) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "CottageWallsPlaneFull15Longest", cottageWallsTriangles, vector{&rayCasterPlaneFull15}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("CottageWallsPlaneFull15Longest", scene.buildAndTraverse());
			}

			// === CottageWalls scene: 1 plane influence area 45 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlaneFull45{ Plane{{7.5,2.9,-7.5}, {-1,0,1}, 8,2.9}, 35, 10000 };
				PlaneRayCaster rayCasterPlaneFull45{ influenceAreaPlaneFull45 }; rayCasterPlaneFull45.generateRays(rng, 1000, true);
				Bvh bvhPlaneFull45{ bvhProperties, influenceAreaPlaneFull45, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlaneFull45) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "CottageWallsPlaneFull45Longest", cottageWallsTriangles, vector{&rayCasterPlaneFull45}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("CottageWallsPlaneFull45Longest", scene.buildAndTraverse());
			}

			// === CottageWalls scene: 1 plane influence area oblique covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlaneFullOblique{ Plane{{10,5,-8}, {-1,-.3,.9}, 7,3.3}, 35, 10000 };
				PlaneRayCaster rayCasterPlaneFullOblique{ influenceAreaPlaneFullOblique }; rayCasterPlaneFullOblique.generateRays(rng, 1000, true);
				Bvh bvhPlaneFullOblique{ bvhProperties, influenceAreaPlaneFullOblique, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlaneFullOblique) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "CottageWallsPlaneFullObliqueLongest", cottageWallsTriangles, vector{&rayCasterPlaneFullOblique}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("CottageWallsPlaneFullObliqueLongest", scene.buildAndTraverse());
			}
		}

		// === Crowd scene: 1 plane full influence area longest split ===
		if constexpr (PLANE_FULL_LONGEST and_or CROWD_SCENE) {
			// === Crowd scene: 1 plane influence area parallel to x covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlaneFullParallel{ Plane{{2,2.2,-8}, {0,0,1}, 7,1.8}, 30, 10000 };
				PlaneRayCaster rayCasterPlaneFullParallel{ influenceAreaPlaneFullParallel }; rayCasterPlaneFullParallel.generateRays(rng, 1000, true);
				Bvh bvhPlaneFullParallel{ bvhProperties, influenceAreaPlaneFullParallel, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlaneFullParallel) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "CrowdPlaneFullParallelLongest", crowdTriangles, vector{&rayCasterPlaneFullParallel}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("CrowdPlaneFullParallelLongest", scene.buildAndTraverse());
			}

			// === Crowd scene: 1 plane influence area 15 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlaneFull15{ Plane{{-2.5,2.2,-8}, {.259,0,.966}, 7.5,1.8}, 30, 10000 };
				PlaneRayCaster rayCasterPlaneFull15{ influenceAreaPlaneFull15 }; rayCasterPlaneFull15.generateRays(rng, 1000, true);
				Bvh bvhPlaneFull15{ bvhProperties, influenceAreaPlaneFull15, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlaneFull15) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "CrowdPlaneFull15Longest", crowdTriangles, vector{&rayCasterPlaneFull15}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("CrowdPlaneFull15Longest", scene.buildAndTraverse());
			}

			// === Crowd scene: 1 plane influence area 45 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlaneFull45{ Plane{{-8,2.2,-8}, {1,0,1}, 9,1.8}, 30, 10000 };
				PlaneRayCaster rayCasterPlaneFull45{ influenceAreaPlaneFull45 }; rayCasterPlaneFull45.generateRays(rng, 1000, true);
				Bvh bvhPlaneFull45{ bvhProperties, influenceAreaPlaneFull45, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlaneFull45) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "CrowdPlaneFull45Longest", crowdTriangles, vector{&rayCasterPlaneFull45}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("CrowdPlaneFull45Longest", scene.buildAndTraverse());
			}

			// === Crowd scene: 1 plane influence area oblique covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlaneFullOblique{ Plane{{-6,7,-8}, {.6,-.4,1}, 7.5,3.4}, 35, 10000 };
				PlaneRayCaster rayCasterPlaneFullOblique{ influenceAreaPlaneFullOblique }; rayCasterPlaneFullOblique.generateRays(rng, 1000, true);
				Bvh bvhPlaneFullOblique{ bvhProperties, influenceAreaPlaneFullOblique, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlaneFullOblique) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "CrowdPlaneFullObliqueLongest", crowdTriangles, vector{&rayCasterPlaneFullOblique}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("CrowdPlaneFullObliqueLongest", scene.buildAndTraverse());
			}
		}

		// === Random100 scene: 1 plane full influence area longest split ===
		if constexpr (PLANE_FULL_LONGEST and_or RANDOM100_SCENE) {
			// === Random100 scene: 1 plane influence area parallel to x covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlaneFullParallel{ Plane{{5,5,-4}, {0,0,1}, 5,5}, 20, 10000 };
				PlaneRayCaster rayCasterPlaneFullParallel{ influenceAreaPlaneFullParallel }; rayCasterPlaneFullParallel.generateRays(rng, 1000, true);
				Bvh bvhPlaneFullParallel{ bvhProperties, influenceAreaPlaneFullParallel, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlaneFullParallel) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Random100PlaneFullParallelLongest", random100Triangles, vector{&rayCasterPlaneFullParallel}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Random100PlaneFullParallelLongest", scene.buildAndTraverse());
			}

			// === Random100 scene: 1 plane influence area 15 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlaneFull15{ Plane{{3.5,5,-4}, {.259,0,.966}, 6.5,5}, 20, 10000 };
				PlaneRayCaster rayCasterPlaneFull15{ influenceAreaPlaneFull15 }; rayCasterPlaneFull15.generateRays(rng, 1000, true);
				Bvh bvhPlaneFull15{ bvhProperties, influenceAreaPlaneFull15, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlaneFull15) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Random100PlaneFull15Longest", random100Triangles, vector{&rayCasterPlaneFull15}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Random100PlaneFull15Longest", scene.buildAndTraverse());
			}

			// === Random100 scene: 1 plane influence area 45 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlaneFull45{ Plane{{-3,5,-3}, {1,0,1}, 7,5}, 20, 10000 };
				PlaneRayCaster rayCasterPlaneFull45{ influenceAreaPlaneFull45 }; rayCasterPlaneFull45.generateRays(rng, 1000, true);
				Bvh bvhPlaneFull45{ bvhProperties, influenceAreaPlaneFull45, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlaneFull45) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Random100PlaneFull45Longest", random100Triangles, vector{&rayCasterPlaneFull45}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Random100PlaneFull45Longest", scene.buildAndTraverse());
			}

			// === Random100 scene: 1 plane influence area oblique covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlaneFullOblique{ Plane{{10,12,-3}, {-.55,-.9,1}, 6.5,6.5}, 20, 10000 };
				PlaneRayCaster rayCasterPlaneFullOblique{ influenceAreaPlaneFullOblique }; rayCasterPlaneFullOblique.generateRays(rng, 1000, true);
				Bvh bvhPlaneFullOblique{ bvhProperties, influenceAreaPlaneFullOblique, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlaneFullOblique) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Random100PlaneFullObliqueLongest", random100Triangles, vector{&rayCasterPlaneFullOblique}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Random100PlaneFullObliqueLongest", scene.buildAndTraverse());
			}
		}

		// === Random1000 scene: 1 plane full influence area longest split ===
		if constexpr (PLANE_FULL_LONGEST and_or RANDOM1000_SCENE) {
			// === Random1000 scene: 1 plane influence area parallel to x covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlaneFullParallel{ Plane{{5,5,-4}, {0,0,1}, 5,5}, 20, 10000 };
				PlaneRayCaster rayCasterPlaneFullParallel{ influenceAreaPlaneFullParallel }; rayCasterPlaneFullParallel.generateRays(rng, 1000, true);
				Bvh bvhPlaneFullParallel{ bvhProperties, influenceAreaPlaneFullParallel, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlaneFullParallel) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Random1000PlaneFullParallelLongest", random1000Triangles, vector{&rayCasterPlaneFullParallel}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Random1000PlaneFullParallelLongest", scene.buildAndTraverse());
			}

			// === Random1000 scene: 1 plane influence area 15 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlaneFull15{ Plane{{3.5,5,-4}, {.259,0,.966}, 6.5,5}, 20, 10000 };
				PlaneRayCaster rayCasterPlaneFull15{ influenceAreaPlaneFull15 }; rayCasterPlaneFull15.generateRays(rng, 1000, true);
				Bvh bvhPlaneFull15{ bvhProperties, influenceAreaPlaneFull15, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlaneFull15) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Random1000PlaneFull15Longest", random1000Triangles, vector{&rayCasterPlaneFull15}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Random1000PlaneFull15Longest", scene.buildAndTraverse());
			}

			// === Random1000 scene: 1 plane influence area 45 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlaneFull45{ Plane{{-3,5,-3}, {1,0,1}, 7,5}, 20, 10000 };
				PlaneRayCaster rayCasterPlaneFull45{ influenceAreaPlaneFull45 }; rayCasterPlaneFull45.generateRays(rng, 1000, true);
				Bvh bvhPlaneFull45{ bvhProperties, influenceAreaPlaneFull45, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlaneFull45) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Random1000PlaneFull45Longest", random1000Triangles, vector{&rayCasterPlaneFull45}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Random1000PlaneFull45Longest", scene.buildAndTraverse());
			}

			// === Random1000 scene: 1 plane influence area oblique covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlaneFullOblique{ Plane{{10,12,-3}, {-.55,-.9,1}, 6.5,6.5}, 20, 10000 };
				PlaneRayCaster rayCasterPlaneFullOblique{ influenceAreaPlaneFullOblique }; rayCasterPlaneFullOblique.generateRays(rng, 1000, true);
				Bvh bvhPlaneFullOblique{ bvhProperties, influenceAreaPlaneFullOblique, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlaneFullOblique) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Random1000PlaneFullObliqueLongest", random1000Triangles, vector{&rayCasterPlaneFullOblique}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Random1000PlaneFullObliqueLongest", scene.buildAndTraverse());
			}
		}
	};

	// POINT FULL PARALLEL
	bvhProperties.splitPlaneQualityThreshold = 0.4f;
	{
		// === Wood scene: 1 point full influence area ===
		if constexpr (POINT_FULL and_or WOOD_SCENE) {
			// === Wood scene: 1 point influence area parallel to x covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPointFullParallel{ Pov{{2.5,.9,0}, {-1,0,0}, 90, 60}, 10, 1, 10000 };
				PointRayCaster rayCasterPointFullParallel{ influenceAreaPointFullParallel }; rayCasterPointFullParallel.generateRays(rng, 1000, true);
				Bvh bvhPointFullParallel{ bvhProperties, influenceAreaPointFullParallel, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPointFullParallel) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "WoodPointFullParallel", woodTriangles, vector{&rayCasterPointFullParallel}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("WoodPointFullParallel", scene.buildAndTraverse());
			}

			// === Wood scene: 1 point influence area 15 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPointFull15{ Pov{{2.5,.8,-.3}, {-.966,0,.259}, 90, 60}, 10, 1, 10000 };
				PointRayCaster rayCasterPointFull15{ influenceAreaPointFull15 }; rayCasterPointFull15.generateRays(rng, 1000, true);
				Bvh bvhPointFull15{ bvhProperties, influenceAreaPointFull15, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPointFull15) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "WoodPointFull15", woodTriangles, vector{&rayCasterPointFull15}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("WoodPointFull15", scene.buildAndTraverse());
			}

			// === Wood scene: 1 point influence area 45 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPointFull45{ Pov{{2.5,.8,-3}, {-1,0,1}, 90, 60}, 10, 1, 10000 };
				PointRayCaster rayCasterPointFull45{ influenceAreaPointFull45 }; rayCasterPointFull45.generateRays(rng, 1000, true);
				Bvh bvhPointFull45{ bvhProperties, influenceAreaPointFull45, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPointFull45) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "WoodPointFull45", woodTriangles, vector{&rayCasterPointFull45}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("WoodPointFull45", scene.buildAndTraverse());
			}

			// === Wood scene: 1 point influence area oblique covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPointFullOblique{ Pov{{2.5,1.5,-1}, {-1,-0.3,0.4}, 90, 60}, 20, 1, 10000 };
				PointRayCaster rayCasterPointFullOblique{ influenceAreaPointFullOblique }; rayCasterPointFullOblique.generateRays(rng, 1000, true);
				Bvh bvhPointFullOblique{ bvhProperties, influenceAreaPointFullOblique, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPointFullOblique) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "WoodPointFullOblique", woodTriangles, vector{&rayCasterPointFullOblique}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("WoodPointFullOblique", scene.buildAndTraverse());
			}
		}

		// === Suzanne scene: 1 point full influence area ===
		if constexpr (POINT_FULL and_or SUZANNE_SCENE) {
			// === Suzanne scene: 1 point influence area parallel to x covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPointFullParallel{ Pov{{0,0,4}, {0,0,-1}, 30, 30}, 10, 2, 10000 };
				PointRayCaster rayCasterPointFullParallel{ influenceAreaPointFullParallel }; rayCasterPointFullParallel.generateRays(rng, 1000, true);
				Bvh bvhPointFullParallel{ bvhProperties, influenceAreaPointFullParallel, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPointFullParallel) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "SuzannePointFullParallel", suzanneTriangles, vector{&rayCasterPointFullParallel}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("SuzannePointFullParallel", scene.buildAndTraverse());
			}
			
			// === Suzanne scene: 1 point influence area 15 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPointFull15{ Pov{{1.3,0,5}, {-.259,0,-.966}, 25, 25}, 10, 2, 10000 };
				PointRayCaster rayCasterPointFull15{ influenceAreaPointFull15 }; rayCasterPointFull15.generateRays(rng, 1000, true);
				Bvh bvhPointFull15{ bvhProperties, influenceAreaPointFull15, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPointFull15) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "SuzannePointFull15", suzanneTriangles, vector{&rayCasterPointFull15}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("SuzannePointFull15", scene.buildAndTraverse());
			}

			// === Suzanne scene: 1 point influence area 45 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPointFull45{ Pov{{3.3,0,3}, {-1,0,-1}, 30, 30}, 10, 2, 10000 };
				PointRayCaster rayCasterPointFull45{ influenceAreaPointFull45 }; rayCasterPointFull45.generateRays(rng, 1000, true);
				Bvh bvhPointFull45{ bvhProperties, influenceAreaPointFull45, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPointFull45) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "SuzannePointFull45", suzanneTriangles, vector{&rayCasterPointFull45}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("SuzannePointFull45", scene.buildAndTraverse());
			}

			// === Suzanne scene: 1 point influence area oblique covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPointFullOblique{ Pov{{2,-1.2,4}, {-.45,.3,-1}, 30, 30}, 10, 2, 10000 };
				PointRayCaster rayCasterPointFullOblique{ influenceAreaPointFullOblique }; rayCasterPointFullOblique.generateRays(rng, 1000, true);
				Bvh bvhPointFullOblique{ bvhProperties, influenceAreaPointFullOblique, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPointFullOblique) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "SuzannePointFullOblique", suzanneTriangles, vector{&rayCasterPointFullOblique}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("SuzannePointFullOblique", scene.buildAndTraverse());
			}
		}

		// === Cottage scene: 1 point full influence area ===
		if constexpr (POINT_FULL and_or COTTAGE_SCENE) {
			// === Cottage scene: 1 point influence area parallel to x covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPointFullParallel{ Pov{{15,2.7,0}, {-1,0,0}, 70, 35}, 35, 1, 10000 };
				PointRayCaster rayCasterPointFullParallel{ influenceAreaPointFullParallel }; rayCasterPointFullParallel.generateRays(rng, 1000, true);
				Bvh bvhPointFullParallel{ bvhProperties, influenceAreaPointFullParallel, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPointFullParallel) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "CottagePointFullParallel", cottageTriangles, vector{&rayCasterPointFullParallel}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("CottagePointFullParallel", scene.buildAndTraverse());
			}

			// === Cottage scene: 1 point influence area 15 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPointFull15{ Pov{{15,2.7,-3}, {-.966,0,.259}, 70, 35}, 35, 1, 10000 };
				PointRayCaster rayCasterPointFull15{ influenceAreaPointFull15 }; rayCasterPointFull15.generateRays(rng, 1000, true);
				Bvh bvhPointFull15{ bvhProperties, influenceAreaPointFull15, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPointFull15) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "CottagePointFull15", cottageTriangles, vector{&rayCasterPointFull15}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("CottagePointFull15", scene.buildAndTraverse());
			}

			// === Cottage scene: 1 point influence area 45 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPointFull45{ Pov{{13,2.5,-13}, {-1,0,1}, 50, 20}, 35, 1, 10000 };
				PointRayCaster rayCasterPointFull45{ influenceAreaPointFull45 }; rayCasterPointFull45.generateRays(rng, 1000, true);
				Bvh bvhPointFull45{ bvhProperties, influenceAreaPointFull45, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPointFull45) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "CottagePointFull45", cottageTriangles, vector{&rayCasterPointFull45}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("CottagePointFull45", scene.buildAndTraverse());
			}

			// === Cottage scene: 1 point influence area oblique covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPointFullOblique{ Pov{{15,7,-11}, {-1,-.45,.75}, 50, 30}, 35, 1, 10000 };
				PointRayCaster rayCasterPointFullOblique{ influenceAreaPointFullOblique }; rayCasterPointFullOblique.generateRays(rng, 1000, true);
				Bvh bvhPointFullOblique{ bvhProperties, influenceAreaPointFullOblique, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPointFullOblique) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "CottagePointFullOblique", cottageTriangles, vector{&rayCasterPointFullOblique}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("CottagePointFullOblique", scene.buildAndTraverse());
			}
		}

		// === CottageWalls scene: 1 point full influence area ===
		if constexpr (POINT_FULL and_or COTTAGE_WALLS_SCENE) {
			// === CottageWalls scene: 1 point influence area parallel to x covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPointFullParallel{ Pov{{15,2.7,0}, {-1,0,0}, 70, 35}, 35, 1, 10000 };
				PointRayCaster rayCasterPointFullParallel{ influenceAreaPointFullParallel }; rayCasterPointFullParallel.generateRays(rng, 1000, true);
				Bvh bvhPointFullParallel{ bvhProperties, influenceAreaPointFullParallel, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPointFullParallel) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "CottageWallsPointFullParallel", cottageWallsTriangles, vector{&rayCasterPointFullParallel}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("CottageWallsPointFullParallel", scene.buildAndTraverse());
			}

			// === CottageWalls scene: 1 point influence area 15 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPointFull15{ Pov{{15,2.7,-3}, {-.966,0,.259}, 70, 35}, 35, 1, 10000 };
				PointRayCaster rayCasterPointFull15{ influenceAreaPointFull15 }; rayCasterPointFull15.generateRays(rng, 1000, true);
				Bvh bvhPointFull15{ bvhProperties, influenceAreaPointFull15, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPointFull15) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "CottageWallsPointFull15", cottageWallsTriangles, vector{&rayCasterPointFull15}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("CottageWallsPointFull15", scene.buildAndTraverse());
			}

			// === CottageWalls scene: 1 point influence area 45 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPointFull45{ Pov{{13,2.5,-13}, {-1,0,1}, 50, 20}, 35, 1, 10000 };
				PointRayCaster rayCasterPointFull45{ influenceAreaPointFull45 }; rayCasterPointFull45.generateRays(rng, 1000, true);
				Bvh bvhPointFull45{ bvhProperties, influenceAreaPointFull45, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPointFull45) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "CottageWallsPointFull45", cottageWallsTriangles, vector{&rayCasterPointFull45}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("CottageWallsPointFull45", scene.buildAndTraverse());
			}

			// === CottageWalls scene: 1 point influence area oblique covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPointFullOblique{ Pov{{15,7,-11}, {-1,-.45,.75}, 50, 30}, 35, 1, 10000 };
				PointRayCaster rayCasterPointFullOblique{ influenceAreaPointFullOblique }; rayCasterPointFullOblique.generateRays(rng, 1000, true);
				Bvh bvhPointFullOblique{ bvhProperties, influenceAreaPointFullOblique, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPointFullOblique) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "CottageWallsPointFullOblique", cottageWallsTriangles, vector{&rayCasterPointFullOblique}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("CottageWallsPointFullOblique", scene.buildAndTraverse());
			}
		}

		// === Crowd scene: 1 point full influence area ===
		if constexpr (POINT_FULL and_or CROWD_SCENE) {
			// === Crowd scene: 1 point influence area parallel to x covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPointFullParallel{ Pov{{2,2.2,-8}, {0,0,1}, 130, 45}, 30, 1, 10000 };
				PointRayCaster rayCasterPointFullParallel{ influenceAreaPointFullParallel }; rayCasterPointFullParallel.generateRays(rng, 1000, true);
				Bvh bvhPointFullParallel{ bvhProperties, influenceAreaPointFullParallel, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPointFullParallel) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "CrowdPointFullParallel", crowdTriangles, vector{&rayCasterPointFullParallel}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("CrowdPointFullParallel", scene.buildAndTraverse());
			}

			// === Crowd scene: 1 point influence area 15 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPointFull15{ Pov{{-2.5,2.2,-8}, {.259,0,.966}, 130, 45}, 30, 1, 10000 };
				PointRayCaster rayCasterPointFull15{ influenceAreaPointFull15 }; rayCasterPointFull15.generateRays(rng, 1000, true);
				Bvh bvhPointFull15{ bvhProperties, influenceAreaPointFull15, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPointFull15) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "CrowdPointFull15", crowdTriangles, vector{&rayCasterPointFull15}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("CrowdPointFull15", scene.buildAndTraverse());
			}

			// === Crowd scene: 1 point influence area 45 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPointFull45{ Pov{{-8,2.2,-8}, {1,0,1}, 130, 45}, 30, 1, 10000 };
				PointRayCaster rayCasterPointFull45{ influenceAreaPointFull45 }; rayCasterPointFull45.generateRays(rng, 1000, true);
				Bvh bvhPointFull45{ bvhProperties, influenceAreaPointFull45, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPointFull45) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "CrowdPointFull45", crowdTriangles, vector{&rayCasterPointFull45}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("CrowdPointFull45", scene.buildAndTraverse());
			}

			// === Crowd scene: 1 point influence area oblique covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPointFullOblique{ Pov{{-6,7,-8}, {.6,-.4,1}, 130, 45}, 35, 1, 10000 };
				PointRayCaster rayCasterPointFullOblique{ influenceAreaPointFullOblique }; rayCasterPointFullOblique.generateRays(rng, 1000, true);
				Bvh bvhPointFullOblique{ bvhProperties, influenceAreaPointFullOblique, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPointFullOblique) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "CrowdPointFullOblique", crowdTriangles, vector{&rayCasterPointFullOblique}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("CrowdPointFullOblique", scene.buildAndTraverse());
			}
		}

		// === Random100 scene: 1 point full influence area ===
		if constexpr (POINT_FULL and_or RANDOM100_SCENE) {
			// === Random100 scene: 1 point influence area parallel to x covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPointFullParallel{ Pov{{5,5,-4}, {0,0,1}, 90, 90}, 20, 1, 10000 };
				PointRayCaster rayCasterPointFullParallel{ influenceAreaPointFullParallel }; rayCasterPointFullParallel.generateRays(rng, 1000, true);
				Bvh bvhPointFullParallel{ bvhProperties, influenceAreaPointFullParallel, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPointFullParallel) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Random100PointFullParallel", random100Triangles, vector{&rayCasterPointFullParallel}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Random100PointFullParallel", scene.buildAndTraverse());
			}

			// === Random100 scene: 1 point influence area 15 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPointFull15{ Pov{{3.5,5,-4}, {.259,0,.966}, 90, 90}, 20, 1, 10000 };
				PointRayCaster rayCasterPointFull15{ influenceAreaPointFull15 }; rayCasterPointFull15.generateRays(rng, 1000, true);
				Bvh bvhPointFull15{ bvhProperties, influenceAreaPointFull15, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPointFull15) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Random100PointFull15", random100Triangles, vector{&rayCasterPointFull15}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Random100PointFull15", scene.buildAndTraverse());
			}

			// === Random100 scene: 1 point influence area 45 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPointFull45{ Pov{{-3,5,-3}, {1,0,1}, 60, 90}, 20, 1, 10000 };
				PointRayCaster rayCasterPointFull45{ influenceAreaPointFull45 }; rayCasterPointFull45.generateRays(rng, 1000, true);
				Bvh bvhPointFull45{ bvhProperties, influenceAreaPointFull45, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPointFull45) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Random100PointFull45", random100Triangles, vector{&rayCasterPointFull45}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Random100PointFull45", scene.buildAndTraverse());
			}

			// === Random100 scene: 1 point influence area oblique covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPointFullOblique{ Pov{{10,12,-3}, {-.55,-.9,1}, 60, 60}, 20, 1, 10000 };
				PointRayCaster rayCasterPointFullOblique{ influenceAreaPointFullOblique }; rayCasterPointFullOblique.generateRays(rng, 1000, true);
				Bvh bvhPointFullOblique{ bvhProperties, influenceAreaPointFullOblique, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPointFullOblique) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Random100PointFullOblique", random100Triangles, vector{&rayCasterPointFullOblique}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Random100PointFullOblique", scene.buildAndTraverse());
			}
		}

		// === Random1000 scene: 1 point full influence area ===
		if constexpr (POINT_FULL and_or RANDOM1000_SCENE) {
			// === Random1000 scene: 1 point influence area parallel to x covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPointFullParallel{ Pov{{5,5,-4}, {0,0,1}, 90, 90}, 20, 1, 10000 };
				PointRayCaster rayCasterPointFullParallel{ influenceAreaPointFullParallel }; rayCasterPointFullParallel.generateRays(rng, 1000, true);
				Bvh bvhPointFullParallel{ bvhProperties, influenceAreaPointFullParallel, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPointFullParallel) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Random1000PointFullParallel", random1000Triangles, vector{&rayCasterPointFullParallel}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Random1000PointFullParallel", scene.buildAndTraverse());
			}

			// === Random1000 scene: 1 point influence area 15 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPointFull15{ Pov{{3.5,5,-4}, {.259,0,.966}, 90, 90}, 20, 1, 10000 };
				PointRayCaster rayCasterPointFull15{ influenceAreaPointFull15 }; rayCasterPointFull15.generateRays(rng, 1000, true);
				Bvh bvhPointFull15{ bvhProperties, influenceAreaPointFull15, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPointFull15) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Random1000PointFull15", random1000Triangles, vector{&rayCasterPointFull15}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Random1000PointFull15", scene.buildAndTraverse());
			}

			// === Random1000 scene: 1 point influence area 45 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPointFull45{ Pov{{-3,5,-3}, {1,0,1}, 60, 90}, 20, 1, 10000 };
				PointRayCaster rayCasterPointFull45{ influenceAreaPointFull45 }; rayCasterPointFull45.generateRays(rng, 1000, true);
				Bvh bvhPointFull45{ bvhProperties, influenceAreaPointFull45, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPointFull45) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Random1000PointFull45", random1000Triangles, vector{&rayCasterPointFull45}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Random1000PointFull45", scene.buildAndTraverse());
			}

			// === Random1000 scene: 1 point influence area oblique covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPointFullOblique{ Pov{{10,12,-3}, {-.55,-.9,1}, 60, 60}, 20, 1, 10000 };
				PointRayCaster rayCasterPointFullOblique{ influenceAreaPointFullOblique }; rayCasterPointFullOblique.generateRays(rng, 1000, true);
				Bvh bvhPointFullOblique{ bvhProperties, influenceAreaPointFullOblique, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPointFullOblique) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Random1000PointFullOblique", random1000Triangles, vector{&rayCasterPointFullOblique}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Random1000PointFullOblique", scene.buildAndTraverse());
			}
		}
	};

	csvTraversal.generateCsv("D:/Users/lapof/Documents/Development/ProjectedAreaHeuristic/Results/ExportedCsv.csv");
	return 0;
}
