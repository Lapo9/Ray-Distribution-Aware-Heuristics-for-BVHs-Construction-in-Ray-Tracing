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
		ACCESSOR("Estimated PAH cost",							AT::TOP_LEVEL,["bvhs"].at(0)["globalInfo"]["pahCost"]),
		ACCESSOR("Estimated SAH cost",							AT::TOP_LEVEL,["bvhs"].at(0)["globalInfo"]["sahCost"]),
		ACCESSOR("Real cost with fallback",						AT::PAH,["cost"]["traversalCostAveragePerRay"]),
		ACCESSOR("Intersections with fallback",					AT::PAH,["total"]["intersectionTests"]["intersectionTestsAveragePerRay"]),
		ACCESSOR("Real cost without fallback",					AT::PAH,["cost"]["traversalCostForBvhPerRay"].at(0).at(1)),
		ACCESSOR("Intersections without fallback",				AT::PAH,["fallback"]["intersectionTests"]["intersectionTestsNonFallbackAveragePerRay"]),
		ACCESSOR("Only fallback cost",							AT::PAH,["cost"]["traversalCostForBvhPerRay"].back().at(1)),
		ACCESSOR("Hit percentage",								AT::PAH,["total"]["hitMiss"]["hitsPercentage"]),
		ACCESSOR("Overlapping % 4",								AT::TOP_LEVEL,["bvhs"].at(0)["globalInfo"]["siblingsOverlappingPercentage4"]),
		ACCESSOR("Overlapping % culled 4",						AT::TOP_LEVEL,["bvhs"].at(0)["globalInfo"]["siblingsOverlappingPercentageCulled4"]),
		ACCESSOR("Overlapping % culled 7",						AT::TOP_LEVEL,["bvhs"].at(0)["globalInfo"]["siblingsOverlappingPercentageCulled7"]),
		ACCESSOR("Overlapping % culled 10",						AT::TOP_LEVEL,["bvhs"].at(0)["globalInfo"]["siblingsOverlappingPercentageCulled10"]),
		ACCESSOR("Overlapping % culled 15",						AT::TOP_LEVEL,["bvhs"].at(0)["globalInfo"]["siblingsOverlappingPercentageCulled15"]),
		ACCESSOR("Overlapping % culled 100",					AT::TOP_LEVEL,["bvhs"].at(0)["globalInfo"]["siblingsOverlappingPercentageCulled100"]),
		ACCESSOR("Max level",									AT::TOP_LEVEL,["bvhs"].at(0)["globalInfo"]["maxLevel"]),
		ACCESSOR("Max triangles per leaf",						AT::TOP_LEVEL,["bvhs"].at(0)["globalInfo"]["properties"]["maxTrianglesPerLeaf"]),
		ACCESSOR("Max non fallback levels",						AT::TOP_LEVEL,["bvhs"].at(0)["globalInfo"]["properties"]["maxNonFallbackLevels"]),
		ACCESSOR("Split plane quality threshold",				AT::TOP_LEVEL,["bvhs"].at(0)["globalInfo"]["properties"]["splitPlaneQualityThreshold"]),
		ACCESSOR("Acceptable children/father hit probability",	AT::TOP_LEVEL,["bvhs"].at(0)["globalInfo"]["properties"]["acceptableChildrenFatherHitProbabilityRatio"]),
		ACCESSOR("Excellent children/father hit probability",	AT::TOP_LEVEL,["bvhs"].at(0)["globalInfo"]["properties"]["excellentChildrenFatherHitProbabilityRatio"]),
		ACCESSOR("Compute cost PAH average",					AT::TOP_LEVEL,["bvhs"].at(0)["totalTiming"]["computeCostTot"]),
		ACCESSOR("Compute cost SAH average",					AT::TOP_LEVEL,["bvhs"].back()["totalTiming"]["computeCostTot"]),	
		ACCESSOR("Compute cost PAH average count",				AT::TOP_LEVEL,["bvhs"].at(0)["totalTiming"]["computeCostCountPerNode"]),
		ACCESSOR("Compute cost SAH average count",				AT::TOP_LEVEL,["bvhs"].back()["totalTiming"]["computeCostCountPerNode"]),
		ACCESSOR("Choose split plane PAH average",				AT::TOP_LEVEL,["bvhs"].at(0)["totalTiming"]["chooseSplittingPlanesTot"]),
		ACCESSOR("Choose split plane SAH average",				AT::TOP_LEVEL,["bvhs"].back()["totalTiming"]["chooseSplittingPlanesTot"]),
		ACCESSOR("Choose split plane PAH average count",		AT::TOP_LEVEL,["bvhs"].at(0)["totalTiming"]["chooseSplittingPlanesCountPerNode"]),
		ACCESSOR("Choose split plane SAH average count",		AT::TOP_LEVEL,["bvhs"].back()["totalTiming"]["chooseSplittingPlanesCountPerNode"])
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
	const vector sponzaTriangles = Triangle::fromObj("D:/Users/lapof/Documents/Development/ProjectedAreaHeuristic/ProjectedAreaHeuristic/exampleData/Sponza.obj");

	//Top level analyzer used by most scenes
	TopLevelOctreeAnalyzer topLevelAnalyzer{
		MAKE_ACTIONS_PAIR(core),
		MAKE_ACTIONS_PAIR(sah),
		MAKE_ACTIONS_PAIR(pah),
		MAKE_ACTIONS_PAIR(levelCount),
		MAKE_ACTIONS_PAIR(triangles),
		MAKE_ACTIONS_PAIR(influenceArea),
		std::pair{ std::function{ analyzerActions::perNode::siblingsOverlapping<4> }, std::function{ analyzerActions::finals::siblingsOverlapping<4> }},
		std::pair{ std::function{ analyzerActions::perNode::siblingsOverlapping<7> }, std::function{ analyzerActions::finals::siblingsOverlapping<7> }},
		std::pair{ std::function{ analyzerActions::perNode::siblingsOverlapping<10> }, std::function{ analyzerActions::finals::siblingsOverlapping<10> }},
		std::pair{ std::function{ analyzerActions::perNode::siblingsOverlapping<15> }, std::function{ analyzerActions::finals::siblingsOverlapping<15> }},
		std::pair{ std::function{ analyzerActions::perNode::siblingsOverlapping<100> }, std::function{ analyzerActions::finals::siblingsOverlapping<100> }},
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
	.splitPlaneQualityThreshold = 0.4f,
	.acceptableChildrenFatherHitProbabilityRatio = 1.3f,
	.excellentChildrenFatherHitProbabilityRatio = 0.9f
	};


	TopLevelOctree::OctreeProperties octreeProperties{
		.maxLevel = 5,
		.conservativeApproach = false
	};

	//fallback BVH used by most top level structures
	Bvh fallbackBvh{ bvhProperties, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesLongest<0.f>, bvhStrategies::shouldStopThresholdOrLevel, "fallback" };
	fallbackBvh.setFallbackComputeCostStrategy(bvhStrategies::computeCostSah);

#define BVH_TESTS 1
#if BVH_TESTS
	// Filter test scenes to run
#define and_or &&
	constexpr bool ALL = false;

	constexpr bool PLANE = ALL || false;
	constexpr bool POINT = ALL || false;
	constexpr bool PLANE_LONGEST = ALL || false;
	constexpr bool POINT_LONGEST = ALL || true;
	constexpr bool PLANE_FACING = ALL || false;
	constexpr bool POINT_FACING = ALL || true;
	constexpr bool PLANE_STANDARD = ALL || false;
	constexpr bool POINT_STANDARD = ALL || false;

	constexpr bool WOOD_SCENE = ALL || false;
	constexpr bool SUZANNE_SCENE = ALL || false;
	constexpr bool COTTAGE_SCENE = ALL || false;
	constexpr bool COTTAGE_WALLS_SCENE = ALL || false;
	constexpr bool CROWD_SCENE = ALL || false;
	constexpr bool RANDOM100_SCENE = ALL || false;
	constexpr bool RANDOM1000_SCENE = ALL || false;
	constexpr bool SPONZA_SCENE = ALL || true;

	constexpr string_view RESULTS_DIRECTORY = "E:/Users/lapof/Documents/Development/ProjectedAreaHeuristic/Results/";
	constexpr float spfhSplitPlaneQualityThreshold = 0.4f;
	constexpr float lsphSplitPlaneQualityThreshold = 0.99f;

	// PLANE PAH+SPFH
	bvhProperties.splitPlaneQualityThreshold = spfhSplitPlaneQualityThreshold;
	{
		// === Wood scene: 1 plane influence area ===
		if constexpr (PLANE and_or WOOD_SCENE) {
			// === Wood scene: 1 plane influence area parallel to x covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlaneParallel{ Plane{{2.5,.9,0}, {-1,0,0}, 1.5,0.9}, 10, 10000 };
				PlaneRayCaster rayCasterPlaneParallel{ influenceAreaPlaneParallel }; rayCasterPlaneParallel.generateRays(rng, 1000, true);
				Bvh bvhPlaneParallel{ bvhProperties, influenceAreaPlaneParallel, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlaneParallel) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Wood__Plane_Parallel_Full", woodTriangles, vector{&rayCasterPlaneParallel}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Wood__Plane_Parallel_Full", scene.buildAndTraverse());
			}

			// === Wood scene: 1 plane influence area 15 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlane15{ Plane{{2.5,.8,-.3}, {-.966,0,.259}, 1.6,1}, 10, 10000 };
				PlaneRayCaster rayCasterPlane15{ influenceAreaPlane15 }; rayCasterPlane15.generateRays(rng, 1000, true);
				Bvh bvhPlane15{ bvhProperties, influenceAreaPlane15, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlane15) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Wood_Plane_15_Full", woodTriangles, vector{&rayCasterPlane15}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Wood_Plane_15_Full", scene.buildAndTraverse());
			}

			// === Wood scene: 1 plane influence area 45 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlane45{ Plane{{2.5,.8,-3}, {-1,0,1}, 1.5,1}, 10, 10000 };
				PlaneRayCaster rayCasterPlane45{ influenceAreaPlane45 }; rayCasterPlane45.generateRays(rng, 1000, true);
				Bvh bvhPlane45{ bvhProperties, influenceAreaPlane45, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlane45) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Wood_Plane_45_Full", woodTriangles, vector{&rayCasterPlane45}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Wood_Plane_45_Full", scene.buildAndTraverse());
			}

			// === Wood scene: 1 plane influence area oblique covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlaneOblique{ Plane{{2.5,1.5,-1}, {-1,-0.3,0.4}, 1.8,1.3}, 20, 10000 };
				PlaneRayCaster rayCasterPlaneOblique{ influenceAreaPlaneOblique }; rayCasterPlaneOblique.generateRays(rng, 1000, true);
				Bvh bvhPlaneOblique{ bvhProperties, influenceAreaPlaneOblique, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlaneOblique) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Wood_Plane_Oblique_Full", woodTriangles, vector{&rayCasterPlaneOblique}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Wood_Plane_Oblique_Full", scene.buildAndTraverse());
			}
		}

		// === Suzanne scene: 1 plane influence area ===
		if constexpr (PLANE and_or SUZANNE_SCENE) {
			// === Suzanne scene: 1 plane influence area parallel to x covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlaneParallel{ Plane{{0,0,2}, {0,0,-1}, .9,.9}, 10, 10000 };
				PlaneRayCaster rayCasterPlaneParallel{ influenceAreaPlaneParallel }; rayCasterPlaneParallel.generateRays(rng, 1000, true);
				Bvh bvhPlaneParallel{ bvhProperties, influenceAreaPlaneParallel, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlaneParallel) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "SuzannePlaneParallel_Full", suzanneTriangles, vector{&rayCasterPlaneParallel}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("SuzannePlaneParallel_Full", scene.buildAndTraverse());
			}

			// === Suzanne scene: 1 plane influence area 15 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlane15{ Plane{{.5,0,2}, {-.259,0,-.966}, 1,.9}, 10, 10000 };
				PlaneRayCaster rayCasterPlane15{ influenceAreaPlane15 }; rayCasterPlane15.generateRays(rng, 1000, true);
				Bvh bvhPlane15{ bvhProperties, influenceAreaPlane15, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlane15) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "SuzannePlane15_Full", suzanneTriangles, vector{&rayCasterPlane15}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("SuzannePlane15_Full", scene.buildAndTraverse());
			}

			// === Suzanne scene: 1 plane influence area 45 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlane45{ Plane{{2,0,2}, {-1,0,-1}, 1.2,.9}, 10, 10000 };
				PlaneRayCaster rayCasterPlane45{ influenceAreaPlane45 }; rayCasterPlane45.generateRays(rng, 1000, true);
				Bvh bvhPlane45{ bvhProperties, influenceAreaPlane45, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlane45) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "SuzannePlane45_Full", suzanneTriangles, vector{&rayCasterPlane45}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("SuzannePlane45_Full", scene.buildAndTraverse());
			}

			// === Suzanne scene: 1 plane influence area oblique covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlaneOblique{ Plane{{1,-.5,2}, {-.45,.3,-1}, 2,1.5}, 10, 10000 };
				PlaneRayCaster rayCasterPlaneOblique{ influenceAreaPlaneOblique }; rayCasterPlaneOblique.generateRays(rng, 1000, true);
				Bvh bvhPlaneOblique{ bvhProperties, influenceAreaPlaneOblique, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlaneOblique) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "SuzannePlaneOblique_Full", suzanneTriangles, vector{&rayCasterPlaneOblique}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("SuzannePlaneOblique_Full", scene.buildAndTraverse());
			}
		}

		// === Cottage scene: 1 plane influence area ===
		if constexpr (PLANE and_or COTTAGE_SCENE) {
			// === Cottage scene: 1 plane influence area parallel to x covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlaneParallel{ Plane{{10,2.8,0}, {-1,0,0}, 6,2.6}, 35, 10000 };
				PlaneRayCaster rayCasterPlaneParallel{ influenceAreaPlaneParallel }; rayCasterPlaneParallel.generateRays(rng, 1000, true);
				Bvh bvhPlaneParallel{ bvhProperties, influenceAreaPlaneParallel, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlaneParallel) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Cottage_Plane_Parallel_Full", cottageTriangles, vector{&rayCasterPlaneParallel}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Cottage_Plane_Parallel_Full", scene.buildAndTraverse());
			}

			// === Cottage scene: 1 plane influence area 15 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlane15{ Plane{{10,2.9,-3}, {-.966,0,.259}, 6,2.9}, 35, 10000 };
				PlaneRayCaster rayCasterPlane15{ influenceAreaPlane15 }; rayCasterPlane15.generateRays(rng, 1000, true);
				Bvh bvhPlane15{ bvhProperties, influenceAreaPlane15, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlane15) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Cottage_Plane_15_Full", cottageTriangles, vector{&rayCasterPlane15}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Cottage_Plane_15_Full", scene.buildAndTraverse());
			}

			// === Cottage scene: 1 plane influence area 45 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlane45{ Plane{{7.5,2.9,-7.5}, {-1,0,1}, 8,2.9}, 35, 10000 };
				PlaneRayCaster rayCasterPlane45{ influenceAreaPlane45 }; rayCasterPlane45.generateRays(rng, 1000, true);
				Bvh bvhPlane45{ bvhProperties, influenceAreaPlane45, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlane45) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Cottage_Plane_45_Full", cottageTriangles, vector{&rayCasterPlane45}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Cottage_Plane_45_Full", scene.buildAndTraverse());
			}

			// === Cottage scene: 1 plane influence area oblique covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlaneOblique{ Plane{{10,5,-8}, {-1,-.3,.9}, 7,3.3}, 35, 10000 };
				PlaneRayCaster rayCasterPlaneOblique{ influenceAreaPlaneOblique }; rayCasterPlaneOblique.generateRays(rng, 1000, true);
				Bvh bvhPlaneOblique{ bvhProperties, influenceAreaPlaneOblique, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlaneOblique) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Cottage_Plane_Oblique_Full", cottageTriangles, vector{&rayCasterPlaneOblique}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Cottage_Plane_Oblique_Full", scene.buildAndTraverse());
			}
		}

		// === CottageWalls scene: 1 plane influence area ===
		if constexpr (PLANE and_or COTTAGE_WALLS_SCENE) {
			// === CottageWalls scene: 1 plane influence area parallel to x covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlaneParallel{ Plane{{10,2.8,0}, {-1,0,0}, 6,2.6}, 35, 10000 };
				PlaneRayCaster rayCasterPlaneParallel{ influenceAreaPlaneParallel }; rayCasterPlaneParallel.generateRays(rng, 1000, true);
				Bvh bvhPlaneParallel{ bvhProperties, influenceAreaPlaneParallel, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlaneParallel) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "CottageWalls_Plane_Parallel_Full", cottageWallsTriangles, vector{&rayCasterPlaneParallel}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("CottageWalls_Plane_Parallel_Full", scene.buildAndTraverse());
			}

			// === CottageWalls scene: 1 plane influence area 15 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlane15{ Plane{{10,2.9,-3}, {-.966,0,.259}, 6,2.9}, 35, 10000 };
				PlaneRayCaster rayCasterPlane15{ influenceAreaPlane15 }; rayCasterPlane15.generateRays(rng, 1000, true);
				Bvh bvhPlane15{ bvhProperties, influenceAreaPlane15, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlane15) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "CottageWalls_Plane_15_Full", cottageWallsTriangles, vector{&rayCasterPlane15}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("CottageWalls_Plane_15_Full", scene.buildAndTraverse());
			}

			// === CottageWalls scene: 1 plane influence area 45 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlane45{ Plane{{7.5,2.9,-7.5}, {-1,0,1}, 8,2.9}, 35, 10000 };
				PlaneRayCaster rayCasterPlane45{ influenceAreaPlane45 }; rayCasterPlane45.generateRays(rng, 1000, true);
				Bvh bvhPlane45{ bvhProperties, influenceAreaPlane45, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlane45) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "CottageWalls_Plane_45_Full", cottageWallsTriangles, vector{&rayCasterPlane45}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("CottageWalls_Plane_45_Full", scene.buildAndTraverse());
			}

			// === CottageWalls scene: 1 plane influence area oblique covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlaneOblique{ Plane{{10,5,-8}, {-1,-.3,.9}, 7,3.3}, 35, 10000 };
				PlaneRayCaster rayCasterPlaneOblique{ influenceAreaPlaneOblique }; rayCasterPlaneOblique.generateRays(rng, 1000, true);
				Bvh bvhPlaneOblique{ bvhProperties, influenceAreaPlaneOblique, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlaneOblique) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "CottageWalls_Plane_Oblique_Full", cottageWallsTriangles, vector{&rayCasterPlaneOblique}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("CottageWalls_Plane_Oblique_Full", scene.buildAndTraverse());
			}
		}

		// === Crowd scene: 1 plane influence area ===
		if constexpr (PLANE and_or CROWD_SCENE) {
			// === Crowd scene: 1 plane influence area parallel to x covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlaneParallel{ Plane{{2,2.2,-8}, {0,0,1}, 7,1.8}, 30, 10000 };
				PlaneRayCaster rayCasterPlaneParallel{ influenceAreaPlaneParallel }; rayCasterPlaneParallel.generateRays(rng, 1000, true);
				Bvh bvhPlaneParallel{ bvhProperties, influenceAreaPlaneParallel, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlaneParallel) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Crowd_Plane_Parallel_Full", crowdTriangles, vector{&rayCasterPlaneParallel}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Crowd_Plane_Parallel_Full", scene.buildAndTraverse());
			}

			// === Crowd scene: 1 plane influence area 15 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlane15{ Plane{{-2.5,2.2,-8}, {.259,0,.966}, 7.5,1.8}, 30, 10000 };
				PlaneRayCaster rayCasterPlane15{ influenceAreaPlane15 }; rayCasterPlane15.generateRays(rng, 1000, true);
				Bvh bvhPlane15{ bvhProperties, influenceAreaPlane15, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlane15) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Crowd_Plane_15_Full", crowdTriangles, vector{&rayCasterPlane15}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Crowd_Plane_15_Full", scene.buildAndTraverse());
			}

			// === Crowd scene: 1 plane influence area 45 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlane45{ Plane{{-8,2.2,-8}, {1,0,1}, 9,1.8}, 30, 10000 };
				PlaneRayCaster rayCasterPlane45{ influenceAreaPlane45 }; rayCasterPlane45.generateRays(rng, 1000, true);
				Bvh bvhPlane45{ bvhProperties, influenceAreaPlane45, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlane45) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Crowd_Plane_45_Full", crowdTriangles, vector{&rayCasterPlane45}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Crowd_Plane_45_Full", scene.buildAndTraverse());
			}

			// === Crowd scene: 1 plane influence area oblique covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlaneOblique{ Plane{{-6,7,-8}, {.6,-.4,1}, 7.5,3.4}, 35, 10000 };
				PlaneRayCaster rayCasterPlaneOblique{ influenceAreaPlaneOblique }; rayCasterPlaneOblique.generateRays(rng, 1000, true);
				Bvh bvhPlaneOblique{ bvhProperties, influenceAreaPlaneOblique, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlaneOblique) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Crowd_Plane_Oblique_Full", crowdTriangles, vector{&rayCasterPlaneOblique}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Crowd_Plane_Oblique_Full", scene.buildAndTraverse());
			}
		}

		// === Random100 scene: 1 plane influence area ===
		if constexpr (PLANE and_or RANDOM100_SCENE) {
			// === Random100 scene: 1 plane influence area parallel to x covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlaneParallel{ Plane{{5,5,-4}, {0,0,1}, 5,5}, 20, 10000 };
				PlaneRayCaster rayCasterPlaneParallel{ influenceAreaPlaneParallel }; rayCasterPlaneParallel.generateRays(rng, 1000, true);
				Bvh bvhPlaneParallel{ bvhProperties, influenceAreaPlaneParallel, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlaneParallel) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Random100_Plane_Parallel_Full", random100Triangles, vector{&rayCasterPlaneParallel}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Random100_Plane_Parallel_Full", scene.buildAndTraverse());
			}

			// === Random100 scene: 1 plane influence area 15 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlane15{ Plane{{3.5,5,-4}, {.259,0,.966}, 6.5,5}, 20, 10000 };
				PlaneRayCaster rayCasterPlane15{ influenceAreaPlane15 }; rayCasterPlane15.generateRays(rng, 1000, true);
				Bvh bvhPlane15{ bvhProperties, influenceAreaPlane15, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlane15) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Random100_Plane_15_Full", random100Triangles, vector{&rayCasterPlane15}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Random100_Plane_15_Full", scene.buildAndTraverse());
			}

			// === Random100 scene: 1 plane influence area 45 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlane45{ Plane{{-3,5,-3}, {1,0,1}, 7,5}, 20, 10000 };
				PlaneRayCaster rayCasterPlane45{ influenceAreaPlane45 }; rayCasterPlane45.generateRays(rng, 1000, true);
				Bvh bvhPlane45{ bvhProperties, influenceAreaPlane45, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlane45) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Random100_Plane_45_Full", random100Triangles, vector{&rayCasterPlane45}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Random100_Plane_45_Full", scene.buildAndTraverse());
			}

			// === Random100 scene: 1 plane influence area oblique covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlaneOblique{ Plane{{10,12,-3}, {-.55,-.9,1}, 6.5,6.5}, 20, 10000 };
				PlaneRayCaster rayCasterPlaneOblique{ influenceAreaPlaneOblique }; rayCasterPlaneOblique.generateRays(rng, 1000, true);
				Bvh bvhPlaneOblique{ bvhProperties, influenceAreaPlaneOblique, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlaneOblique) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Random100_Plane_Oblique_Full", random100Triangles, vector{&rayCasterPlaneOblique}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Random100_Plane_Oblique_Full", scene.buildAndTraverse());
			}
		}

		// === Random1000 scene: 1 plane influence area ===
		if constexpr (PLANE and_or RANDOM1000_SCENE) {
			// === Random1000 scene: 1 plane influence area parallel to x covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlaneParallel{ Plane{{5,5,-4}, {0,0,1}, 5,5}, 20, 10000 };
				PlaneRayCaster rayCasterPlaneParallel{ influenceAreaPlaneParallel }; rayCasterPlaneParallel.generateRays(rng, 1000, true);
				Bvh bvhPlaneParallel{ bvhProperties, influenceAreaPlaneParallel, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlaneParallel) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Random1000_Plane_Parallel_Full", random1000Triangles, vector{&rayCasterPlaneParallel}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Random1000_Plane_Parallel_Full", scene.buildAndTraverse());
			}

			// === Random1000 scene: 1 plane influence area 15 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlane15{ Plane{{3.5,5,-4}, {.259,0,.966}, 6.5,5}, 20, 10000 };
				PlaneRayCaster rayCasterPlane15{ influenceAreaPlane15 }; rayCasterPlane15.generateRays(rng, 1000, true);
				Bvh bvhPlane15{ bvhProperties, influenceAreaPlane15, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlane15) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Random1000_Plane_15_Full", random1000Triangles, vector{&rayCasterPlane15}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Random1000_Plane_15_Full", scene.buildAndTraverse());
			}

			// === Random1000 scene: 1 plane influence area 45 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlane45{ Plane{{-3,5,-3}, {1,0,1}, 7,5}, 20, 10000 };
				PlaneRayCaster rayCasterPlane45{ influenceAreaPlane45 }; rayCasterPlane45.generateRays(rng, 1000, true);
				Bvh bvhPlane45{ bvhProperties, influenceAreaPlane45, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlane45) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Random1000_Plane_45_Full", random1000Triangles, vector{&rayCasterPlane45}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Random1000_Plane_45_Full", scene.buildAndTraverse());
			}

			// === Random1000 scene: 1 plane influence area oblique covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlaneOblique{ Plane{{10,12,-3}, {-.55,-.9,1}, 6.5,6.5}, 20, 10000 };
				PlaneRayCaster rayCasterPlaneOblique{ influenceAreaPlaneOblique }; rayCasterPlaneOblique.generateRays(rng, 1000, true);
				Bvh bvhPlaneOblique{ bvhProperties, influenceAreaPlaneOblique, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlaneOblique) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Random1000_Plane_Oblique_Full", random1000Triangles, vector{&rayCasterPlaneOblique}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Random1000_Plane_Oblique_Full", scene.buildAndTraverse());
			}
		}

		// === Sponza scene: 1 plane influence area ===
		if constexpr (PLANE and_or SPONZA_SCENE) {
			// === Sponza scene: 1 plane influence area parallel to x covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlaneParallel{ Plane{{-9,11,-1}, {1,0,0}, 16, 11}, 50, 10000 };
				PlaneRayCaster rayCasterPlaneParallel{ influenceAreaPlaneParallel }; rayCasterPlaneParallel.generateRays(rng, 1000, true);
				Bvh bvhPlaneParallel{ bvhProperties, influenceAreaPlaneParallel, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlaneParallel) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Sponza_Plane_Parallel_Full", sponzaTriangles, vector{&rayCasterPlaneParallel}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Sponza_Plane_Parallel_Full", scene.buildAndTraverse());
			}
	
			// === Sponza scene: 1 plane influence area 15 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlane15{ Plane{{-12,11,0.5}, {.966,0,-.259}, 16,11}, 60, 10000 };
				PlaneRayCaster rayCasterPlane15{ influenceAreaPlane15 }; rayCasterPlane15.generateRays(rng, 1000, true);
				Bvh bvhPlane15{ bvhProperties, influenceAreaPlane15, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlane15) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Sponza_Plane_15_Full", sponzaTriangles, vector{&rayCasterPlane15}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Sponza_Plane_15_Full", scene.buildAndTraverse());
			}

			// === Sponza scene: 1 plane influence area 45 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlane45{ Plane{{-10,11,18}, {1,0,-1}, 25,11}, 70, 10000 };
				PlaneRayCaster rayCasterPlane45{ influenceAreaPlane45 }; rayCasterPlane45.generateRays(rng, 1000, true);
				Bvh bvhPlane45{ bvhProperties, influenceAreaPlane45, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlane45) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Sponza_Plane_45_Full", sponzaTriangles, vector{&rayCasterPlane45}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Sponza_Plane_45_Full", scene.buildAndTraverse());
			}

			// === Sponza scene: 1 plane influence area oblique covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlaneOblique{ Plane{{-12,10,7.2}, {.84,0.26,-.48}, 16,11}, 70, 10000 };
				PlaneRayCaster rayCasterPlaneOblique{ influenceAreaPlaneOblique }; rayCasterPlaneOblique.generateRays(rng, 1000, true);
				Bvh bvhPlaneOblique{ bvhProperties, influenceAreaPlaneOblique, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlaneOblique) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Sponza_Plane_Oblique_Full", sponzaTriangles, vector{&rayCasterPlaneOblique}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Sponza_Plane_Oblique_Full", scene.buildAndTraverse());
			}	
		}
	};

	// POINT PAH+SPFH
	bvhProperties.splitPlaneQualityThreshold = spfhSplitPlaneQualityThreshold;
	{
		// === Wood scene: 1 point influence area ===
		if constexpr (POINT and_or WOOD_SCENE) {
			// === Wood scene: 1 point influence area parallel to x covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPointParallel{ Pov{{2.5,.9,0}, {-1,0,0}, 90, 60}, 10, 1, 10000 };
				PointRayCaster rayCasterPointParallel{ influenceAreaPointParallel }; rayCasterPointParallel.generateRays(rng, 1000, true);
				Bvh bvhPointParallel{ bvhProperties, influenceAreaPointParallel, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPointParallel) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Wood_Point_Parallel_Full", woodTriangles, vector{&rayCasterPointParallel}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Wood_Point_Parallel_Full", scene.buildAndTraverse());
			}

			// === Wood scene: 1 point influence area 15 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPoint15{ Pov{{2.5,.8,-.3}, {-.966,0,.259}, 90, 60}, 10, 1, 10000 };
				PointRayCaster rayCasterPoint15{ influenceAreaPoint15 }; rayCasterPoint15.generateRays(rng, 1000, true);
				Bvh bvhPoint15{ bvhProperties, influenceAreaPoint15, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPoint15) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Wood_Point_15_Full", woodTriangles, vector{&rayCasterPoint15}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Wood_Point_15_Full", scene.buildAndTraverse());
			}

			// === Wood scene: 1 point influence area 45 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPoint45{ Pov{{2.5,.8,-3}, {-1,0,1}, 90, 60}, 10, 1, 10000 };
				PointRayCaster rayCasterPoint45{ influenceAreaPoint45 }; rayCasterPoint45.generateRays(rng, 1000, true);
				Bvh bvhPoint45{ bvhProperties, influenceAreaPoint45, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPoint45) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Wood_Point_45_Full", woodTriangles, vector{&rayCasterPoint45}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Wood_Point_45_Full", scene.buildAndTraverse());
			}

			// === Wood scene: 1 point influence area oblique covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPointOblique{ Pov{{2.5,1.5,-1}, {-1,-0.3,0.4}, 90, 60}, 20, 1, 10000 };
				PointRayCaster rayCasterPointOblique{ influenceAreaPointOblique }; rayCasterPointOblique.generateRays(rng, 1000, true);
				Bvh bvhPointOblique{ bvhProperties, influenceAreaPointOblique, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPointOblique) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Wood_Point_Oblique_Full", woodTriangles, vector{&rayCasterPointOblique}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Wood_Point_Oblique_Full", scene.buildAndTraverse());
			}
		}

		// === Suzanne scene: 1 point influence area ===
		if constexpr (POINT and_or SUZANNE_SCENE) {
			// === Suzanne scene: 1 point influence area parallel to x covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPointParallel{ Pov{{0,0,4}, {0,0,-1}, 30, 30}, 10, 2, 10000 };
				PointRayCaster rayCasterPointParallel{ influenceAreaPointParallel }; rayCasterPointParallel.generateRays(rng, 1000, true);
				Bvh bvhPointParallel{ bvhProperties, influenceAreaPointParallel, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPointParallel) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "SuzannePointParallel_Full", suzanneTriangles, vector{&rayCasterPointParallel}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("SuzannePointParallel_Full", scene.buildAndTraverse());
			}
			
			// === Suzanne scene: 1 point influence area 15 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPoint15{ Pov{{1.3,0,5}, {-.259,0,-.966}, 25, 25}, 10, 2, 10000 };
				PointRayCaster rayCasterPoint15{ influenceAreaPoint15 }; rayCasterPoint15.generateRays(rng, 1000, true);
				Bvh bvhPoint15{ bvhProperties, influenceAreaPoint15, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPoint15) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "SuzannePoint15_Full", suzanneTriangles, vector{&rayCasterPoint15}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("SuzannePoint15_Full", scene.buildAndTraverse());
			}

			// === Suzanne scene: 1 point influence area 45 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPoint45{ Pov{{3.3,0,3}, {-1,0,-1}, 30, 30}, 10, 2, 10000 };
				PointRayCaster rayCasterPoint45{ influenceAreaPoint45 }; rayCasterPoint45.generateRays(rng, 1000, true);
				Bvh bvhPoint45{ bvhProperties, influenceAreaPoint45, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPoint45) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "SuzannePoint45_Full", suzanneTriangles, vector{&rayCasterPoint45}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("SuzannePoint45_Full", scene.buildAndTraverse());
			}

			// === Suzanne scene: 1 point influence area oblique covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPointOblique{ Pov{{2,-1.2,4}, {-.45,.3,-1}, 30, 30}, 10, 2, 10000 };
				PointRayCaster rayCasterPointOblique{ influenceAreaPointOblique }; rayCasterPointOblique.generateRays(rng, 1000, true);
				Bvh bvhPointOblique{ bvhProperties, influenceAreaPointOblique, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPointOblique) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "SuzannePointOblique_Full", suzanneTriangles, vector{&rayCasterPointOblique}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("SuzannePointOblique_Full", scene.buildAndTraverse());
			}
		}

		// === Cottage scene: 1 point influence area ===
		if constexpr (POINT and_or COTTAGE_SCENE) {
			// === Cottage scene: 1 point influence area parallel to x covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPointParallel{ Pov{{15,2.7,0}, {-1,0,0}, 70, 35}, 35, 1, 10000 };
				PointRayCaster rayCasterPointParallel{ influenceAreaPointParallel }; rayCasterPointParallel.generateRays(rng, 1000, true);
				Bvh bvhPointParallel{ bvhProperties, influenceAreaPointParallel, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPointParallel) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Cottage_Point_Parallel_Full", cottageTriangles, vector{&rayCasterPointParallel}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Cottage_Point_Parallel_Full", scene.buildAndTraverse());
			}

			// === Cottage scene: 1 point influence area 15 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPoint15{ Pov{{15,2.7,-3}, {-.966,0,.259}, 70, 35}, 35, 1, 10000 };
				PointRayCaster rayCasterPoint15{ influenceAreaPoint15 }; rayCasterPoint15.generateRays(rng, 1000, true);
				Bvh bvhPoint15{ bvhProperties, influenceAreaPoint15, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPoint15) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Cottage_Point_15_Full", cottageTriangles, vector{&rayCasterPoint15}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Cottage_Point_15_Full", scene.buildAndTraverse());
			}

			// === Cottage scene: 1 point influence area 45 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPoint45{ Pov{{13,2.5,-13}, {-1,0,1}, 50, 20}, 35, 1, 10000 };
				PointRayCaster rayCasterPoint45{ influenceAreaPoint45 }; rayCasterPoint45.generateRays(rng, 1000, true);
				Bvh bvhPoint45{ bvhProperties, influenceAreaPoint45, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPoint45) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Cottage_Point_45_Full", cottageTriangles, vector{&rayCasterPoint45}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Cottage_Point_45_Full", scene.buildAndTraverse());
			}

			// === Cottage scene: 1 point influence area oblique covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPointOblique{ Pov{{15,7,-11}, {-1,-.45,.75}, 50, 30}, 35, 1, 10000 };
				PointRayCaster rayCasterPointOblique{ influenceAreaPointOblique }; rayCasterPointOblique.generateRays(rng, 1000, true);
				Bvh bvhPointOblique{ bvhProperties, influenceAreaPointOblique, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPointOblique) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Cottage_Point_Oblique_Full", cottageTriangles, vector{&rayCasterPointOblique}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Cottage_Point_Oblique_Full", scene.buildAndTraverse());
			}
		}

		// === CottageWalls scene: 1 point influence area ===
		if constexpr (POINT and_or COTTAGE_WALLS_SCENE) {
			// === CottageWalls scene: 1 point influence area parallel to x covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPointParallel{ Pov{{15,2.7,0}, {-1,0,0}, 70, 35}, 35, 1, 10000 };
				PointRayCaster rayCasterPointParallel{ influenceAreaPointParallel }; rayCasterPointParallel.generateRays(rng, 1000, true);
				Bvh bvhPointParallel{ bvhProperties, influenceAreaPointParallel, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPointParallel) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "CottageWalls_Point_Parallel_Full", cottageWallsTriangles, vector{&rayCasterPointParallel}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("CottageWalls_Point_Parallel_Full", scene.buildAndTraverse());
			}

			// === CottageWalls scene: 1 point influence area 15 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPoint15{ Pov{{15,2.7,-3}, {-.966,0,.259}, 70, 35}, 35, 1, 10000 };
				PointRayCaster rayCasterPoint15{ influenceAreaPoint15 }; rayCasterPoint15.generateRays(rng, 1000, true);
				Bvh bvhPoint15{ bvhProperties, influenceAreaPoint15, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPoint15) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "CottageWalls_Point_15_Full", cottageWallsTriangles, vector{&rayCasterPoint15}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("CottageWalls_Point_15_Full", scene.buildAndTraverse());
			}

			// === CottageWalls scene: 1 point influence area 45 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPoint45{ Pov{{13,2.5,-13}, {-1,0,1}, 50, 20}, 35, 1, 10000 };
				PointRayCaster rayCasterPoint45{ influenceAreaPoint45 }; rayCasterPoint45.generateRays(rng, 1000, true);
				Bvh bvhPoint45{ bvhProperties, influenceAreaPoint45, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPoint45) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "CottageWalls_Point_45_Full", cottageWallsTriangles, vector{&rayCasterPoint45}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("CottageWalls_Point_45_Full", scene.buildAndTraverse());
			}

			// === CottageWalls scene: 1 point influence area oblique covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPointOblique{ Pov{{15,7,-11}, {-1,-.45,.75}, 50, 30}, 35, 1, 10000 };
				PointRayCaster rayCasterPointOblique{ influenceAreaPointOblique }; rayCasterPointOblique.generateRays(rng, 1000, true);
				Bvh bvhPointOblique{ bvhProperties, influenceAreaPointOblique, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPointOblique) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "CottageWalls_Point_Oblique_Full", cottageWallsTriangles, vector{&rayCasterPointOblique}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("CottageWalls_Point_Oblique_Full", scene.buildAndTraverse());
			}
		}

		// === Crowd scene: 1 point influence area ===
		if constexpr (POINT and_or CROWD_SCENE) {
			// === Crowd scene: 1 point influence area parallel to x covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPointParallel{ Pov{{2,2.2,-8}, {0,0,1}, 130, 45}, 30, 1, 10000 };
				PointRayCaster rayCasterPointParallel{ influenceAreaPointParallel }; rayCasterPointParallel.generateRays(rng, 1000, true);
				Bvh bvhPointParallel{ bvhProperties, influenceAreaPointParallel, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPointParallel) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Crowd_Point_Parallel_Full", crowdTriangles, vector{&rayCasterPointParallel}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Crowd_Point_Parallel_Full", scene.buildAndTraverse());
			}

			// === Crowd scene: 1 point influence area 15 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPoint15{ Pov{{-2.5,2.2,-8}, {.259,0,.966}, 130, 45}, 30, 1, 10000 };
				PointRayCaster rayCasterPoint15{ influenceAreaPoint15 }; rayCasterPoint15.generateRays(rng, 1000, true);
				Bvh bvhPoint15{ bvhProperties, influenceAreaPoint15, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPoint15) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Crowd_Point_15_Full", crowdTriangles, vector{&rayCasterPoint15}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Crowd_Point_15_Full", scene.buildAndTraverse());
			}

			// === Crowd scene: 1 point influence area 45 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPoint45{ Pov{{-8,2.2,-8}, {1,0,1}, 130, 45}, 30, 1, 10000 };
				PointRayCaster rayCasterPoint45{ influenceAreaPoint45 }; rayCasterPoint45.generateRays(rng, 1000, true);
				Bvh bvhPoint45{ bvhProperties, influenceAreaPoint45, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPoint45) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Crowd_Point_45_Full", crowdTriangles, vector{&rayCasterPoint45}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Crowd_Point_45_Full", scene.buildAndTraverse());
			}

			// === Crowd scene: 1 point influence area oblique covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPointOblique{ Pov{{-6,7,-8}, {.6,-.4,1}, 130, 45}, 35, 1, 10000 };
				PointRayCaster rayCasterPointOblique{ influenceAreaPointOblique }; rayCasterPointOblique.generateRays(rng, 1000, true);
				Bvh bvhPointOblique{ bvhProperties, influenceAreaPointOblique, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPointOblique) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Crowd_Point_Oblique_Full", crowdTriangles, vector{&rayCasterPointOblique}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Crowd_Point_Oblique_Full", scene.buildAndTraverse());
			}
		}

		// === Random100 scene: 1 point influence area ===
		if constexpr (POINT and_or RANDOM100_SCENE) {
			// === Random100 scene: 1 point influence area parallel to x covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPointParallel{ Pov{{5,5,-4}, {0,0,1}, 90, 90}, 20, 1, 10000 };
				PointRayCaster rayCasterPointParallel{ influenceAreaPointParallel }; rayCasterPointParallel.generateRays(rng, 1000, true);
				Bvh bvhPointParallel{ bvhProperties, influenceAreaPointParallel, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPointParallel) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Random100_Point_Parallel_Full", random100Triangles, vector{&rayCasterPointParallel}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Random100_Point_Parallel_Full", scene.buildAndTraverse());
			}

			// === Random100 scene: 1 point influence area 15 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPoint15{ Pov{{3.5,5,-4}, {.259,0,.966}, 90, 90}, 20, 1, 10000 };
				PointRayCaster rayCasterPoint15{ influenceAreaPoint15 }; rayCasterPoint15.generateRays(rng, 1000, true);
				Bvh bvhPoint15{ bvhProperties, influenceAreaPoint15, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPoint15) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Random100_Point_15_Full", random100Triangles, vector{&rayCasterPoint15}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Random100_Point_15_Full", scene.buildAndTraverse());
			}

			// === Random100 scene: 1 point influence area 45 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPoint45{ Pov{{-3,5,-3}, {1,0,1}, 60, 90}, 20, 1, 10000 };
				PointRayCaster rayCasterPoint45{ influenceAreaPoint45 }; rayCasterPoint45.generateRays(rng, 1000, true);
				Bvh bvhPoint45{ bvhProperties, influenceAreaPoint45, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPoint45) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Random100_Point_45_Full", random100Triangles, vector{&rayCasterPoint45}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Random100_Point_45_Full", scene.buildAndTraverse());
			}

			// === Random100 scene: 1 point influence area oblique covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPointOblique{ Pov{{10,12,-3}, {-.55,-.9,1}, 60, 60}, 20, 1, 10000 };
				PointRayCaster rayCasterPointOblique{ influenceAreaPointOblique }; rayCasterPointOblique.generateRays(rng, 1000, true);
				Bvh bvhPointOblique{ bvhProperties, influenceAreaPointOblique, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPointOblique) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Random100_Point_Oblique_Full", random100Triangles, vector{&rayCasterPointOblique}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Random100_Point_Oblique_Full", scene.buildAndTraverse());
			}
		}

		// === Random1000 scene: 1 point influence area ===
		if constexpr (POINT and_or RANDOM1000_SCENE) {
			// === Random1000 scene: 1 point influence area parallel to x covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPointParallel{ Pov{{5,5,-4}, {0,0,1}, 90, 90}, 20, 1, 10000 };
				PointRayCaster rayCasterPointParallel{ influenceAreaPointParallel }; rayCasterPointParallel.generateRays(rng, 1000, true);
				Bvh bvhPointParallel{ bvhProperties, influenceAreaPointParallel, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPointParallel) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Random1000_Point_Parallel_Full", random1000Triangles, vector{&rayCasterPointParallel}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Random1000_Point_Parallel_Full", scene.buildAndTraverse());
			}

			// === Random1000 scene: 1 point influence area 15 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPoint15{ Pov{{3.5,5,-4}, {.259,0,.966}, 90, 90}, 20, 1, 10000 };
				PointRayCaster rayCasterPoint15{ influenceAreaPoint15 }; rayCasterPoint15.generateRays(rng, 1000, true);
				Bvh bvhPoint15{ bvhProperties, influenceAreaPoint15, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPoint15) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Random1000_Point_15_Full", random1000Triangles, vector{&rayCasterPoint15}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Random1000_Point_15_Full", scene.buildAndTraverse());
			}

			// === Random1000 scene: 1 point influence area 45 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPoint45{ Pov{{-3,5,-3}, {1,0,1}, 60, 90}, 20, 1, 10000 };
				PointRayCaster rayCasterPoint45{ influenceAreaPoint45 }; rayCasterPoint45.generateRays(rng, 1000, true);
				Bvh bvhPoint45{ bvhProperties, influenceAreaPoint45, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPoint45) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Random1000_Point_45_Full", random1000Triangles, vector{&rayCasterPoint45}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Random1000_Point_45_Full", scene.buildAndTraverse());
			}

			// === Random1000 scene: 1 point influence area oblique covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPointOblique{ Pov{{10,12,-3}, {-.55,-.9,1}, 60, 60}, 20, 1, 10000 };
				PointRayCaster rayCasterPointOblique{ influenceAreaPointOblique }; rayCasterPointOblique.generateRays(rng, 1000, true);
				Bvh bvhPointOblique{ bvhProperties, influenceAreaPointOblique, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPointOblique) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Random1000_Point_Oblique_Full", random1000Triangles, vector{&rayCasterPointOblique}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Random1000_Point_Oblique_Full", scene.buildAndTraverse());
			}
		}

		// === Sponza scene: 1 point influence area ===
		if constexpr (POINT and_or SPONZA_SCENE) {
			// === Sponza scene: 1 point influence area parallel to x covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPointParallel{ Pov{{-9,11,-1}, {1,0,0}, 70, 50}, 50, 1, 10000 };
				PointRayCaster rayCasterPointParallel{ influenceAreaPointParallel }; rayCasterPointParallel.generateRays(rng, 1000, true);
				Bvh bvhPointParallel{ bvhProperties, influenceAreaPointParallel, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPointParallel) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Sponza_Point_Parallel_Full", sponzaTriangles, vector{&rayCasterPointParallel}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Sponza_Point_Parallel_Full", scene.buildAndTraverse());
			}

			// === Sponza scene: 1 point influence area 15 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPoint15{ Pov{{-12,11,0.5}, {.966,0,-.259}, 70,50}, 60,1, 10000 };
				PointRayCaster rayCasterPoint15{ influenceAreaPoint15 }; rayCasterPoint15.generateRays(rng, 1000, true);
				Bvh bvhPoint15{ bvhProperties, influenceAreaPoint15, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPoint15) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Sponza_Point_15_Full", sponzaTriangles, vector{&rayCasterPoint15}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Sponza_Point_15_Full", scene.buildAndTraverse());
			}

			// === Sponza scene: 1 point influence area 45 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPoint45{ Pov{{-10,11,18}, {1,0,-1}, 90,50}, 70,1, 10000 };
				PointRayCaster rayCasterPoint45{ influenceAreaPoint45 }; rayCasterPoint45.generateRays(rng, 1000, true);
				Bvh bvhPoint45{ bvhProperties, influenceAreaPoint45, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPoint45) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Sponza_Point_45_Full", sponzaTriangles, vector{&rayCasterPoint45}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Sponza_Point_45_Full", scene.buildAndTraverse());
			}

			// === Sponza scene: 1 point influence area oblique covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPointOblique{ Pov{{-12,10,7.2}, {.84,0.26,-.48}, 70,50}, 70,1, 10000 };
				PointRayCaster rayCasterPointOblique{ influenceAreaPointOblique }; rayCasterPointOblique.generateRays(rng, 1000, true);
				Bvh bvhPointOblique{ bvhProperties, influenceAreaPointOblique, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPointOblique) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Sponza_Point_Oblique_Full", sponzaTriangles, vector{&rayCasterPointOblique}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Sponza_Point_Oblique_Full", scene.buildAndTraverse());
			}
		}
	};

	// PLANE PAH+LSPH
	bvhProperties.splitPlaneQualityThreshold = lsphSplitPlaneQualityThreshold;
	{
		// === Wood scene: 1 plane influence area longest split ===
		if constexpr (PLANE_LONGEST and_or WOOD_SCENE) {
			// === Wood scene: 1 plane influence area parallel to x covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlaneParallel{ Plane{{2.5,.9,0}, {-1,0,0}, 1.5,0.9}, 10, 10000 };
				PlaneRayCaster rayCasterPlaneParallel{ influenceAreaPlaneParallel }; rayCasterPlaneParallel.generateRays(rng, 1000, true);
				Bvh bvhPlaneParallel{ bvhProperties, influenceAreaPlaneParallel, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlaneParallel) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Wood_Plane_Parallel_Longest", woodTriangles, vector{&rayCasterPlaneParallel}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Wood_Plane_Parallel_Longest", scene.buildAndTraverse());
			}

			// === Wood scene: 1 plane influence area 15 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlane15{ Plane{{2.5,.8,-.3}, {-.966,0,.259}, 1.6,1}, 10, 10000 };
				PlaneRayCaster rayCasterPlane15{ influenceAreaPlane15 }; rayCasterPlane15.generateRays(rng, 1000, true);
				Bvh bvhPlane15{ bvhProperties, influenceAreaPlane15, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlane15) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Wood_Plane_15_Longest", woodTriangles, vector{&rayCasterPlane15}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Wood_Plane_15_Longest", scene.buildAndTraverse());
			}

			// === Wood scene: 1 plane influence area 45 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlane45{ Plane{{2.5,.8,-3}, {-1,0,1}, 1.5,1}, 10, 10000 };
				PlaneRayCaster rayCasterPlane45{ influenceAreaPlane45 }; rayCasterPlane45.generateRays(rng, 1000, true);
				Bvh bvhPlane45{ bvhProperties, influenceAreaPlane45, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlane45) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Wood_Plane_45_Longest", woodTriangles, vector{&rayCasterPlane45}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Wood_Plane_45_Longest", scene.buildAndTraverse());
			}

			// === Wood scene: 1 plane influence area oblique covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlaneOblique{ Plane{{2.5,1.5,-1}, {-1,-0.3,0.4}, 1.8,1.3}, 20, 10000 };
				PlaneRayCaster rayCasterPlaneOblique{ influenceAreaPlaneOblique }; rayCasterPlaneOblique.generateRays(rng, 1000, true);
				Bvh bvhPlaneOblique{ bvhProperties, influenceAreaPlaneOblique, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlaneOblique) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Wood_Plane_Oblique_Longest", woodTriangles, vector{&rayCasterPlaneOblique}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Wood_Plane_Oblique_Longest", scene.buildAndTraverse());
			}
		}

		// === Suzanne scene: 1 plane influence area longest split ===
		if constexpr (PLANE_LONGEST and_or SUZANNE_SCENE) {
			// === Suzanne scene: 1 plane influence area parallel to x covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlaneParallel{ Plane{{0,0,2}, {0,0,-1}, .9,.9}, 10, 10000 };
				PlaneRayCaster rayCasterPlaneParallel{ influenceAreaPlaneParallel }; rayCasterPlaneParallel.generateRays(rng, 1000, true);
				Bvh bvhPlaneParallel{ bvhProperties, influenceAreaPlaneParallel, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlaneParallel) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "SuzannePlaneParallel_Longest", suzanneTriangles, vector{&rayCasterPlaneParallel}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("SuzannePlaneParallel_Longest", scene.buildAndTraverse());
			}

			// === Suzanne scene: 1 plane influence area 15 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlane15{ Plane{{.5,0,2}, {-.259,0,-.966}, 1,.9}, 10, 10000 };
				PlaneRayCaster rayCasterPlane15{ influenceAreaPlane15 }; rayCasterPlane15.generateRays(rng, 1000, true);
				Bvh bvhPlane15{ bvhProperties, influenceAreaPlane15, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlane15) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "SuzannePlane15_Longest", suzanneTriangles, vector{&rayCasterPlane15}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("SuzannePlane15_Longest", scene.buildAndTraverse());
			}

			// === Suzanne scene: 1 plane influence area 45 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlane45{ Plane{{2,0,2}, {-1,0,-1}, 1.2,.9}, 10, 10000 };
				PlaneRayCaster rayCasterPlane45{ influenceAreaPlane45 }; rayCasterPlane45.generateRays(rng, 1000, true);
				Bvh bvhPlane45{ bvhProperties, influenceAreaPlane45, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlane45) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "SuzannePlane45_Longest", suzanneTriangles, vector{&rayCasterPlane45}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("SuzannePlane45_Longest", scene.buildAndTraverse());
			}

			// === Suzanne scene: 1 plane influence area oblique covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlaneOblique{ Plane{{1,-.5,2}, {-.45,.3,-1}, .9,.9}, 10, 10000 };
				PlaneRayCaster rayCasterPlaneOblique{ influenceAreaPlaneOblique }; rayCasterPlaneOblique.generateRays(rng, 1000, true);
				Bvh bvhPlaneOblique{ bvhProperties, influenceAreaPlaneOblique, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlaneOblique) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "SuzannePlaneOblique_Longest", suzanneTriangles, vector{&rayCasterPlaneOblique}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("SuzannePlaneOblique_Longest", scene.buildAndTraverse());
			}
		}

		// === Cottage scene: 1 plane influence area longest split ===
		if constexpr (PLANE_LONGEST and_or COTTAGE_SCENE) {
			// === Cottage scene: 1 plane influence area parallel to x covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlaneParallel{ Plane{{10,2.8,0}, {-1,0,0}, 6,2.6}, 35, 10000 };
				PlaneRayCaster rayCasterPlaneParallel{ influenceAreaPlaneParallel }; rayCasterPlaneParallel.generateRays(rng, 1000, true);
				Bvh bvhPlaneParallel{ bvhProperties, influenceAreaPlaneParallel, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlaneParallel) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Cottage_Plane_Parallel_Longest", cottageTriangles, vector{&rayCasterPlaneParallel}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Cottage_Plane_Parallel_Longest", scene.buildAndTraverse());
			}

			// === Cottage scene: 1 plane influence area 15 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlane15{ Plane{{10,2.9,-3}, {-.966,0,.259}, 6,2.9}, 35, 10000 };
				PlaneRayCaster rayCasterPlane15{ influenceAreaPlane15 }; rayCasterPlane15.generateRays(rng, 1000, true);
				Bvh bvhPlane15{ bvhProperties, influenceAreaPlane15, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlane15) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Cottage_Plane_15_Longest", cottageTriangles, vector{&rayCasterPlane15}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Cottage_Plane_15_Longest", scene.buildAndTraverse());
			}

			// === Cottage scene: 1 plane influence area 45 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlane45{ Plane{{7.5,2.9,-7.5}, {-1,0,1}, 8,2.9}, 35, 10000 };
				PlaneRayCaster rayCasterPlane45{ influenceAreaPlane45 }; rayCasterPlane45.generateRays(rng, 1000, true);
				Bvh bvhPlane45{ bvhProperties, influenceAreaPlane45, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlane45) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Cottage_Plane_45_Longest", cottageTriangles, vector{&rayCasterPlane45}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Cottage_Plane_45_Longest", scene.buildAndTraverse());
			}

			// === Cottage scene: 1 plane influence area oblique covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlaneOblique{ Plane{{10,5,-8}, {-1,-.3,.9}, 7,3.3}, 35, 10000 };
				PlaneRayCaster rayCasterPlaneOblique{ influenceAreaPlaneOblique }; rayCasterPlaneOblique.generateRays(rng, 1000, true);
				Bvh bvhPlaneOblique{ bvhProperties, influenceAreaPlaneOblique, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlaneOblique) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Cottage_Plane_Oblique_Longest", cottageTriangles, vector{&rayCasterPlaneOblique}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Cottage_Plane_Oblique_Longest", scene.buildAndTraverse());
			}
		}

		// === CottageWalls scene: 1 plane influence area longest split ===
		if constexpr (PLANE_LONGEST and_or COTTAGE_WALLS_SCENE) {
			// === CottageWalls scene: 1 plane influence area parallel to x covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlaneParallel{ Plane{{10,2.8,0}, {-1,0,0}, 6,2.6}, 35, 10000 };
				PlaneRayCaster rayCasterPlaneParallel{ influenceAreaPlaneParallel }; rayCasterPlaneParallel.generateRays(rng, 1000, true);
				Bvh bvhPlaneParallel{ bvhProperties, influenceAreaPlaneParallel, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlaneParallel) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "CottageWalls_Plane_Parallel_Longest", cottageWallsTriangles, vector{&rayCasterPlaneParallel}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("CottageWalls_Plane_Parallel_Longest", scene.buildAndTraverse());
			}

			// === CottageWalls scene: 1 plane influence area 15 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlane15{ Plane{{10,2.9,-3}, {-.966,0,.259}, 6,2.9}, 35, 10000 };
				PlaneRayCaster rayCasterPlane15{ influenceAreaPlane15 }; rayCasterPlane15.generateRays(rng, 1000, true);
				Bvh bvhPlane15{ bvhProperties, influenceAreaPlane15, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlane15) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "CottageWalls_Plane_15_Longest", cottageWallsTriangles, vector{&rayCasterPlane15}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("CottageWalls_Plane_15_Longest", scene.buildAndTraverse());
			}

			// === CottageWalls scene: 1 plane influence area 45 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlane45{ Plane{{7.5,2.9,-7.5}, {-1,0,1}, 8,2.9}, 35, 10000 };
				PlaneRayCaster rayCasterPlane45{ influenceAreaPlane45 }; rayCasterPlane45.generateRays(rng, 1000, true);
				Bvh bvhPlane45{ bvhProperties, influenceAreaPlane45, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlane45) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "CottageWalls_Plane_45_Longest", cottageWallsTriangles, vector{&rayCasterPlane45}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("CottageWalls_Plane_45_Longest", scene.buildAndTraverse());
			}

			// === CottageWalls scene: 1 plane influence area oblique covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlaneOblique{ Plane{{10,5,-8}, {-1,-.3,.9}, 7,3.3}, 35, 10000 };
				PlaneRayCaster rayCasterPlaneOblique{ influenceAreaPlaneOblique }; rayCasterPlaneOblique.generateRays(rng, 1000, true);
				Bvh bvhPlaneOblique{ bvhProperties, influenceAreaPlaneOblique, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlaneOblique) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "CottageWalls_Plane_Oblique_Longest", cottageWallsTriangles, vector{&rayCasterPlaneOblique}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("CottageWalls_Plane_Oblique_Longest", scene.buildAndTraverse());
			}
		}

		// === Crowd scene: 1 plane influence area longest split ===
		if constexpr (PLANE_LONGEST and_or CROWD_SCENE) {
			// === Crowd scene: 1 plane influence area parallel to x covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlaneParallel{ Plane{{2,2.2,-8}, {0,0,1}, 7,1.8}, 30, 10000 };
				PlaneRayCaster rayCasterPlaneParallel{ influenceAreaPlaneParallel }; rayCasterPlaneParallel.generateRays(rng, 1000, true);
				Bvh bvhPlaneParallel{ bvhProperties, influenceAreaPlaneParallel, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlaneParallel) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Crowd_Plane_Parallel_Longest", crowdTriangles, vector{&rayCasterPlaneParallel}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Crowd_Plane_Parallel_Longest", scene.buildAndTraverse());
			}

			// === Crowd scene: 1 plane influence area 15 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlane15{ Plane{{-2.5,2.2,-8}, {.259,0,.966}, 7.5,1.8}, 30, 10000 };
				PlaneRayCaster rayCasterPlane15{ influenceAreaPlane15 }; rayCasterPlane15.generateRays(rng, 1000, true);
				Bvh bvhPlane15{ bvhProperties, influenceAreaPlane15, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlane15) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Crowd_Plane_15_Longest", crowdTriangles, vector{&rayCasterPlane15}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Crowd_Plane_15_Longest", scene.buildAndTraverse());
			}

			// === Crowd scene: 1 plane influence area 45 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlane45{ Plane{{-8,2.2,-8}, {1,0,1}, 9,1.8}, 30, 10000 };
				PlaneRayCaster rayCasterPlane45{ influenceAreaPlane45 }; rayCasterPlane45.generateRays(rng, 1000, true);
				Bvh bvhPlane45{ bvhProperties, influenceAreaPlane45, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlane45) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Crowd_Plane_45_Longest", crowdTriangles, vector{&rayCasterPlane45}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Crowd_Plane_45_Longest", scene.buildAndTraverse());
			}

			// === Crowd scene: 1 plane influence area oblique covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlaneOblique{ Plane{{-6,7,-8}, {.6,-.4,1}, 7.5,3.4}, 35, 10000 };
				PlaneRayCaster rayCasterPlaneOblique{ influenceAreaPlaneOblique }; rayCasterPlaneOblique.generateRays(rng, 1000, true);
				Bvh bvhPlaneOblique{ bvhProperties, influenceAreaPlaneOblique, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlaneOblique) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Crowd_Plane_Oblique_Longest", crowdTriangles, vector{&rayCasterPlaneOblique}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Crowd_Plane_Oblique_Longest", scene.buildAndTraverse());
			}
		}

		// === Random100 scene: 1 plane influence area longest split ===
		if constexpr (PLANE_LONGEST and_or RANDOM100_SCENE) {
			// === Random100 scene: 1 plane influence area parallel to x covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlaneParallel{ Plane{{5,5,-4}, {0,0,1}, 5,5}, 20, 10000 };
				PlaneRayCaster rayCasterPlaneParallel{ influenceAreaPlaneParallel }; rayCasterPlaneParallel.generateRays(rng, 1000, true);
				Bvh bvhPlaneParallel{ bvhProperties, influenceAreaPlaneParallel, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlaneParallel) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Random100_Plane_Parallel_Longest", random100Triangles, vector{&rayCasterPlaneParallel}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Random100_Plane_Parallel_Longest", scene.buildAndTraverse());
			}

			// === Random100 scene: 1 plane influence area 15 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlane15{ Plane{{3.5,5,-4}, {.259,0,.966}, 6.5,5}, 20, 10000 };
				PlaneRayCaster rayCasterPlane15{ influenceAreaPlane15 }; rayCasterPlane15.generateRays(rng, 1000, true);
				Bvh bvhPlane15{ bvhProperties, influenceAreaPlane15, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlane15) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Random100_Plane_15_Longest", random100Triangles, vector{&rayCasterPlane15}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Random100_Plane_15_Longest", scene.buildAndTraverse());
			}

			// === Random100 scene: 1 plane influence area 45 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlane45{ Plane{{-3,5,-3}, {1,0,1}, 7,5}, 20, 10000 };
				PlaneRayCaster rayCasterPlane45{ influenceAreaPlane45 }; rayCasterPlane45.generateRays(rng, 1000, true);
				Bvh bvhPlane45{ bvhProperties, influenceAreaPlane45, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlane45) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Random100_Plane_45_Longest", random100Triangles, vector{&rayCasterPlane45}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Random100_Plane_45_Longest", scene.buildAndTraverse());
			}

			// === Random100 scene: 1 plane influence area oblique covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlaneOblique{ Plane{{10,12,-3}, {-.55,-.9,1}, 6.5,6.5}, 20, 10000 };
				PlaneRayCaster rayCasterPlaneOblique{ influenceAreaPlaneOblique }; rayCasterPlaneOblique.generateRays(rng, 1000, true);
				Bvh bvhPlaneOblique{ bvhProperties, influenceAreaPlaneOblique, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlaneOblique) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Random100_Plane_Oblique_Longest", random100Triangles, vector{&rayCasterPlaneOblique}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Random100_Plane_Oblique_Longest", scene.buildAndTraverse());
			}
		}

		// === Random1000 scene: 1 plane influence area longest split ===
		if constexpr (PLANE_LONGEST and_or RANDOM1000_SCENE) {
			// === Random1000 scene: 1 plane influence area parallel to x covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlaneParallel{ Plane{{5,5,-4}, {0,0,1}, 5,5}, 20, 10000 };
				PlaneRayCaster rayCasterPlaneParallel{ influenceAreaPlaneParallel }; rayCasterPlaneParallel.generateRays(rng, 1000, true);
				Bvh bvhPlaneParallel{ bvhProperties, influenceAreaPlaneParallel, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlaneParallel) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Random1000_Plane_Parallel_Longest", random1000Triangles, vector{&rayCasterPlaneParallel}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Random1000_Plane_Parallel_Longest", scene.buildAndTraverse());
			}

			// === Random1000 scene: 1 plane influence area 15 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlane15{ Plane{{3.5,5,-4}, {.259,0,.966}, 6.5,5}, 20, 10000 };
				PlaneRayCaster rayCasterPlane15{ influenceAreaPlane15 }; rayCasterPlane15.generateRays(rng, 1000, true);
				Bvh bvhPlane15{ bvhProperties, influenceAreaPlane15, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlane15) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Random1000_Plane_15_Longest", random1000Triangles, vector{&rayCasterPlane15}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Random1000_Plane_15_Longest", scene.buildAndTraverse());
			}

			// === Random1000 scene: 1 plane influence area 45 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlane45{ Plane{{-3,5,-3}, {1,0,1}, 7,5}, 20, 10000 };
				PlaneRayCaster rayCasterPlane45{ influenceAreaPlane45 }; rayCasterPlane45.generateRays(rng, 1000, true);
				Bvh bvhPlane45{ bvhProperties, influenceAreaPlane45, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlane45) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Random1000_Plane_45_Longest", random1000Triangles, vector{&rayCasterPlane45}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Random1000_Plane_45_Longest", scene.buildAndTraverse());
			}

			// === Random1000 scene: 1 plane influence area oblique covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlaneOblique{ Plane{{10,12,-3}, {-.55,-.9,1}, 6.5,6.5}, 20, 10000 };
				PlaneRayCaster rayCasterPlaneOblique{ influenceAreaPlaneOblique }; rayCasterPlaneOblique.generateRays(rng, 1000, true);
				Bvh bvhPlaneOblique{ bvhProperties, influenceAreaPlaneOblique, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlaneOblique) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Random1000_Plane_Oblique_Longest", random1000Triangles, vector{&rayCasterPlaneOblique}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Random1000_Plane_Oblique_Longest", scene.buildAndTraverse());
			}
		}

		// === Sponza scene: 1 plane influence area longest split ===
		if constexpr (PLANE_LONGEST and_or SPONZA_SCENE) {
			// === Sponza scene: 1 plane influence area parallel to x covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlaneParallel{ Plane{{-9,11,-1}, {1,0,0}, 16, 11}, 50, 10000 };
				PlaneRayCaster rayCasterPlaneParallel{ influenceAreaPlaneParallel }; rayCasterPlaneParallel.generateRays(rng, 1000, true);
				Bvh bvhPlaneParallel{ bvhProperties, influenceAreaPlaneParallel, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlaneParallel) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Sponza_Plane_Parallel_Longest", sponzaTriangles, vector{&rayCasterPlaneParallel}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Sponza_Plane_Parallel_Longest", scene.buildAndTraverse());
			}

			// === Sponza scene: 1 plane influence area 15 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlane15{ Plane{{-12,11,0.5}, {.966,0,-.259}, 16,11}, 60, 10000 };
				PlaneRayCaster rayCasterPlane15{ influenceAreaPlane15 }; rayCasterPlane15.generateRays(rng, 1000, true);
				Bvh bvhPlane15{ bvhProperties, influenceAreaPlane15, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlane15) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Sponza_Plane_15_Longest", sponzaTriangles, vector{&rayCasterPlane15}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Sponza_Plane_15_Longest", scene.buildAndTraverse());
			}

			// === Sponza scene: 1 plane influence area 45 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlane45{ Plane{{-10,11,18}, {1,0,-1}, 25,11}, 70, 10000 };
				PlaneRayCaster rayCasterPlane45{ influenceAreaPlane45 }; rayCasterPlane45.generateRays(rng, 1000, true);
				Bvh bvhPlane45{ bvhProperties, influenceAreaPlane45, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlane45) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Sponza_Plane_45_Longest", sponzaTriangles, vector{&rayCasterPlane45}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Sponza_Plane_45_Longest", scene.buildAndTraverse());
			}

			// === Sponza scene: 1 plane influence area oblique covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlaneOblique{ Plane{{-12,10,7.2}, {.84,0.26,-.48}, 16,11}, 70, 10000 };
				PlaneRayCaster rayCasterPlaneOblique{ influenceAreaPlaneOblique }; rayCasterPlaneOblique.generateRays(rng, 1000, true);
				Bvh bvhPlaneOblique{ bvhProperties, influenceAreaPlaneOblique, PAH_STRATEGY, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlaneOblique) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Sponza_Plane_Oblique_Longest", sponzaTriangles, vector{&rayCasterPlaneOblique}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Sponza_Plane_Oblique_Longest", scene.buildAndTraverse());
			}
		}
	};

	// POINT PAH+LSPH
	bvhProperties.splitPlaneQualityThreshold = lsphSplitPlaneQualityThreshold;
	{
		// === Wood scene: 1 point influence area ===
		if constexpr (POINT_LONGEST and_or WOOD_SCENE) {
			// === Wood scene: 1 point influence area parallel to x covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPointParallel{ Pov{{2.5,.9,0}, {-1,0,0}, 90, 60}, 10, 1, 10000 };
				PointRayCaster rayCasterPointParallel{ influenceAreaPointParallel }; rayCasterPointParallel.generateRays(rng, 1000, true);
				Bvh bvhPointParallel{ bvhProperties, influenceAreaPointParallel, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPointParallel) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Wood_Point_Parallel_Longest", woodTriangles, vector{&rayCasterPointParallel}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Wood_Point_Parallel_Longest", scene.buildAndTraverse());
			}

			// === Wood scene: 1 point influence area 15 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPoint15{ Pov{{2.5,.8,-.3}, {-.966,0,.259}, 90, 60}, 10, 1, 10000 };
				PointRayCaster rayCasterPoint15{ influenceAreaPoint15 }; rayCasterPoint15.generateRays(rng, 1000, true);
				Bvh bvhPoint15{ bvhProperties, influenceAreaPoint15, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPoint15) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Wood_Point_15_Longest", woodTriangles, vector{&rayCasterPoint15}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Wood_Point_15_Longest", scene.buildAndTraverse());
			}

			// === Wood scene: 1 point influence area 45 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPoint45{ Pov{{2.5,.8,-3}, {-1,0,1}, 90, 60}, 10, 1, 10000 };
				PointRayCaster rayCasterPoint45{ influenceAreaPoint45 }; rayCasterPoint45.generateRays(rng, 1000, true);
				Bvh bvhPoint45{ bvhProperties, influenceAreaPoint45, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPoint45) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Wood_Point_45_Longest", woodTriangles, vector{&rayCasterPoint45}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Wood_Point_45_Longest", scene.buildAndTraverse());
			}

			// === Wood scene: 1 point influence area oblique covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPointOblique{ Pov{{2.5,1.5,-1}, {-1,-0.3,0.4}, 90, 60}, 20, 1, 10000 };
				PointRayCaster rayCasterPointOblique{ influenceAreaPointOblique }; rayCasterPointOblique.generateRays(rng, 1000, true);
				Bvh bvhPointOblique{ bvhProperties, influenceAreaPointOblique, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPointOblique) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Wood_Point_Oblique_Longest", woodTriangles, vector{&rayCasterPointOblique}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Wood_Point_Oblique_Longest", scene.buildAndTraverse());
			}
		}

		// === Suzanne scene: 1 point influence area ===
		if constexpr (POINT_LONGEST and_or SUZANNE_SCENE) {
			// === Suzanne scene: 1 point influence area parallel to x covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPointParallel{ Pov{{0,0,4}, {0,0,-1}, 30, 30}, 10, 2, 10000 };
				PointRayCaster rayCasterPointParallel{ influenceAreaPointParallel }; rayCasterPointParallel.generateRays(rng, 1000, true);
				Bvh bvhPointParallel{ bvhProperties, influenceAreaPointParallel, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPointParallel) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "SuzannePointParallel_Longest", suzanneTriangles, vector{&rayCasterPointParallel}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("SuzannePointParallel_Longest", scene.buildAndTraverse());
			}

			// === Suzanne scene: 1 point influence area 15 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPoint15{ Pov{{1.3,0,5}, {-.259,0,-.966}, 25, 25}, 10, 2, 10000 };
				PointRayCaster rayCasterPoint15{ influenceAreaPoint15 }; rayCasterPoint15.generateRays(rng, 1000, true);
				Bvh bvhPoint15{ bvhProperties, influenceAreaPoint15, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPoint15) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "SuzannePoint15_Longest", suzanneTriangles, vector{&rayCasterPoint15}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("SuzannePoint15_Longest", scene.buildAndTraverse());
			}

			// === Suzanne scene: 1 point influence area 45 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPoint45{ Pov{{3.3,0,3}, {-1,0,-1}, 30, 30}, 10, 2, 10000 };
				PointRayCaster rayCasterPoint45{ influenceAreaPoint45 }; rayCasterPoint45.generateRays(rng, 1000, true);
				Bvh bvhPoint45{ bvhProperties, influenceAreaPoint45, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPoint45) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "SuzannePoint45_Longest", suzanneTriangles, vector{&rayCasterPoint45}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("SuzannePoint45_Longest", scene.buildAndTraverse());
			}

			// === Suzanne scene: 1 point influence area oblique covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPointOblique{ Pov{{2,-1.2,4}, {-.45,.3,-1}, 30, 30}, 10, 2, 10000 };
				PointRayCaster rayCasterPointOblique{ influenceAreaPointOblique }; rayCasterPointOblique.generateRays(rng, 1000, true);
				Bvh bvhPointOblique{ bvhProperties, influenceAreaPointOblique, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPointOblique) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "SuzannePointOblique_Longest", suzanneTriangles, vector{&rayCasterPointOblique}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("SuzannePointOblique_Longest", scene.buildAndTraverse());
			}
		}

		// === Cottage scene: 1 point influence area ===
		if constexpr (POINT_LONGEST and_or COTTAGE_SCENE) {
			// === Cottage scene: 1 point influence area parallel to x covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPointParallel{ Pov{{15,2.7,0}, {-1,0,0}, 70, 35}, 35, 1, 10000 };
				PointRayCaster rayCasterPointParallel{ influenceAreaPointParallel }; rayCasterPointParallel.generateRays(rng, 1000, true);
				Bvh bvhPointParallel{ bvhProperties, influenceAreaPointParallel, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPointParallel) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Cottage_Point_Parallel_Longest", cottageTriangles, vector{&rayCasterPointParallel}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Cottage_Point_Parallel_Longest", scene.buildAndTraverse());
			}

			// === Cottage scene: 1 point influence area 15 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPoint15{ Pov{{15,2.7,-3}, {-.966,0,.259}, 70, 35}, 35, 1, 10000 };
				PointRayCaster rayCasterPoint15{ influenceAreaPoint15 }; rayCasterPoint15.generateRays(rng, 1000, true);
				Bvh bvhPoint15{ bvhProperties, influenceAreaPoint15, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPoint15) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Cottage_Point_15_Longest", cottageTriangles, vector{&rayCasterPoint15}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Cottage_Point_15_Longest", scene.buildAndTraverse());
			}

			// === Cottage scene: 1 point influence area 45 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPoint45{ Pov{{13,2.5,-13}, {-1,0,1}, 50, 20}, 35, 1, 10000 };
				PointRayCaster rayCasterPoint45{ influenceAreaPoint45 }; rayCasterPoint45.generateRays(rng, 1000, true);
				Bvh bvhPoint45{ bvhProperties, influenceAreaPoint45, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPoint45) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Cottage_Point_45_Longest", cottageTriangles, vector{&rayCasterPoint45}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Cottage_Point_45_Longest", scene.buildAndTraverse());
			}

			// === Cottage scene: 1 point influence area oblique covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPointOblique{ Pov{{15,7,-11}, {-1,-.45,.75}, 50, 30}, 35, 1, 10000 };
				PointRayCaster rayCasterPointOblique{ influenceAreaPointOblique }; rayCasterPointOblique.generateRays(rng, 1000, true);
				Bvh bvhPointOblique{ bvhProperties, influenceAreaPointOblique, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPointOblique) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Cottage_Point_Oblique_Longest", cottageTriangles, vector{&rayCasterPointOblique}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Cottage_Point_Oblique_Longest", scene.buildAndTraverse());
			}
		}

		// === CottageWalls scene: 1 point influence area ===
		if constexpr (POINT_LONGEST and_or COTTAGE_WALLS_SCENE) {
			// === CottageWalls scene: 1 point influence area parallel to x covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPointParallel{ Pov{{15,2.7,0}, {-1,0,0}, 70, 35}, 35, 1, 10000 };
				PointRayCaster rayCasterPointParallel{ influenceAreaPointParallel }; rayCasterPointParallel.generateRays(rng, 1000, true);
				Bvh bvhPointParallel{ bvhProperties, influenceAreaPointParallel, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPointParallel) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "CottageWalls_Point_Parallel_Longest", cottageWallsTriangles, vector{&rayCasterPointParallel}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("CottageWalls_Point_Parallel_Longest", scene.buildAndTraverse());
			}

			// === CottageWalls scene: 1 point influence area 15 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPoint15{ Pov{{15,2.7,-3}, {-.966,0,.259}, 70, 35}, 35, 1, 10000 };
				PointRayCaster rayCasterPoint15{ influenceAreaPoint15 }; rayCasterPoint15.generateRays(rng, 1000, true);
				Bvh bvhPoint15{ bvhProperties, influenceAreaPoint15, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPoint15) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "CottageWalls_Point_15_Longest", cottageWallsTriangles, vector{&rayCasterPoint15}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("CottageWalls_Point_15_Longest", scene.buildAndTraverse());
			}

			// === CottageWalls scene: 1 point influence area 45 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPoint45{ Pov{{13,2.5,-13}, {-1,0,1}, 50, 20}, 35, 1, 10000 };
				PointRayCaster rayCasterPoint45{ influenceAreaPoint45 }; rayCasterPoint45.generateRays(rng, 1000, true);
				Bvh bvhPoint45{ bvhProperties, influenceAreaPoint45, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPoint45) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "CottageWalls_Point_45_Longest", cottageWallsTriangles, vector{&rayCasterPoint45}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("CottageWalls_Point_45_Longest", scene.buildAndTraverse());
			}

			// === CottageWalls scene: 1 point influence area oblique covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPointOblique{ Pov{{15,7,-11}, {-1,-.45,.75}, 50, 30}, 35, 1, 10000 };
				PointRayCaster rayCasterPointOblique{ influenceAreaPointOblique }; rayCasterPointOblique.generateRays(rng, 1000, true);
				Bvh bvhPointOblique{ bvhProperties, influenceAreaPointOblique, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPointOblique) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "CottageWalls_Point_Oblique_Longest", cottageWallsTriangles, vector{&rayCasterPointOblique}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("CottageWalls_Point_Oblique_Longest", scene.buildAndTraverse());
			}
		}

		// === Crowd scene: 1 point influence area ===
		if constexpr (POINT_LONGEST and_or CROWD_SCENE) {
			// === Crowd scene: 1 point influence area parallel to x covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPointParallel{ Pov{{2,2.2,-8}, {0,0,1}, 130, 45}, 30, 1, 10000 };
				PointRayCaster rayCasterPointParallel{ influenceAreaPointParallel }; rayCasterPointParallel.generateRays(rng, 1000, true);
				Bvh bvhPointParallel{ bvhProperties, influenceAreaPointParallel, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPointParallel) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Crowd_Point_Parallel_Longest", crowdTriangles, vector{&rayCasterPointParallel}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Crowd_Point_Parallel_Longest", scene.buildAndTraverse());
			}

			// === Crowd scene: 1 point influence area 15 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPoint15{ Pov{{-2.5,2.2,-8}, {.259,0,.966}, 130, 45}, 30, 1, 10000 };
				PointRayCaster rayCasterPoint15{ influenceAreaPoint15 }; rayCasterPoint15.generateRays(rng, 1000, true);
				Bvh bvhPoint15{ bvhProperties, influenceAreaPoint15, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPoint15) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Crowd_Point_15_Longest", crowdTriangles, vector{&rayCasterPoint15}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Crowd_Point_15_Longest", scene.buildAndTraverse());
			}

			// === Crowd scene: 1 point influence area 45 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPoint45{ Pov{{-8,2.2,-8}, {1,0,1}, 130, 45}, 30, 1, 10000 };
				PointRayCaster rayCasterPoint45{ influenceAreaPoint45 }; rayCasterPoint45.generateRays(rng, 1000, true);
				Bvh bvhPoint45{ bvhProperties, influenceAreaPoint45, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPoint45) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Crowd_Point_45_Longest", crowdTriangles, vector{&rayCasterPoint45}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Crowd_Point_45_Longest", scene.buildAndTraverse());
			}

			// === Crowd scene: 1 point influence area oblique covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPointOblique{ Pov{{-6,7,-8}, {.6,-.4,1}, 130, 45}, 35, 1, 10000 };
				PointRayCaster rayCasterPointOblique{ influenceAreaPointOblique }; rayCasterPointOblique.generateRays(rng, 1000, true);
				Bvh bvhPointOblique{ bvhProperties, influenceAreaPointOblique, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPointOblique) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Crowd_Point_Oblique_Longest", crowdTriangles, vector{&rayCasterPointOblique}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Crowd_Point_Oblique_Longest", scene.buildAndTraverse());
			}
		}

		// === Random100 scene: 1 point influence area ===
		if constexpr (POINT_LONGEST and_or RANDOM100_SCENE) {
			// === Random100 scene: 1 point influence area parallel to x covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPointParallel{ Pov{{5,5,-4}, {0,0,1}, 90, 90}, 20, 1, 10000 };
				PointRayCaster rayCasterPointParallel{ influenceAreaPointParallel }; rayCasterPointParallel.generateRays(rng, 1000, true);
				Bvh bvhPointParallel{ bvhProperties, influenceAreaPointParallel, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPointParallel) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Random100_Point_Parallel_Longest", random100Triangles, vector{&rayCasterPointParallel}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Random100_Point_Parallel_Longest", scene.buildAndTraverse());
			}

			// === Random100 scene: 1 point influence area 15 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPoint15{ Pov{{3.5,5,-4}, {.259,0,.966}, 90, 90}, 20, 1, 10000 };
				PointRayCaster rayCasterPoint15{ influenceAreaPoint15 }; rayCasterPoint15.generateRays(rng, 1000, true);
				Bvh bvhPoint15{ bvhProperties, influenceAreaPoint15, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPoint15) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Random100_Point_15_Longest", random100Triangles, vector{&rayCasterPoint15}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Random100_Point_15_Longest", scene.buildAndTraverse());
			}

			// === Random100 scene: 1 point influence area 45 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPoint45{ Pov{{-3,5,-3}, {1,0,1}, 60, 90}, 20, 1, 10000 };
				PointRayCaster rayCasterPoint45{ influenceAreaPoint45 }; rayCasterPoint45.generateRays(rng, 1000, true);
				Bvh bvhPoint45{ bvhProperties, influenceAreaPoint45, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPoint45) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Random100_Point_45_Longest", random100Triangles, vector{&rayCasterPoint45}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Random100_Point_45_Longest", scene.buildAndTraverse());
			}

			// === Random100 scene: 1 point influence area oblique covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPointOblique{ Pov{{10,12,-3}, {-.55,-.9,1}, 60, 60}, 20, 1, 10000 };
				PointRayCaster rayCasterPointOblique{ influenceAreaPointOblique }; rayCasterPointOblique.generateRays(rng, 1000, true);
				Bvh bvhPointOblique{ bvhProperties, influenceAreaPointOblique, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPointOblique) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Random100_Point_Oblique_Longest", random100Triangles, vector{&rayCasterPointOblique}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Random100_Point_Oblique_Longest", scene.buildAndTraverse());
			}
		}

		// === Random1000 scene: 1 point influence area ===
		if constexpr (POINT_LONGEST and_or RANDOM1000_SCENE) {
			// === Random1000 scene: 1 point influence area parallel to x covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPointParallel{ Pov{{5,5,-4}, {0,0,1}, 90, 90}, 20, 1, 10000 };
				PointRayCaster rayCasterPointParallel{ influenceAreaPointParallel }; rayCasterPointParallel.generateRays(rng, 1000, true);
				Bvh bvhPointParallel{ bvhProperties, influenceAreaPointParallel, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPointParallel) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Random1000_Point_Parallel_Longest", random1000Triangles, vector{&rayCasterPointParallel}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Random1000_Point_Parallel_Longest", scene.buildAndTraverse());
			}

			// === Random1000 scene: 1 point influence area 15 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPoint15{ Pov{{3.5,5,-4}, {.259,0,.966}, 90, 90}, 20, 1, 10000 };
				PointRayCaster rayCasterPoint15{ influenceAreaPoint15 }; rayCasterPoint15.generateRays(rng, 1000, true);
				Bvh bvhPoint15{ bvhProperties, influenceAreaPoint15, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPoint15) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Random1000_Point_15_Longest", random1000Triangles, vector{&rayCasterPoint15}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Random1000_Point_15_Longest", scene.buildAndTraverse());
			}

			// === Random1000 scene: 1 point influence area 45 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPoint45{ Pov{{-3,5,-3}, {1,0,1}, 60, 90}, 20, 1, 10000 };
				PointRayCaster rayCasterPoint45{ influenceAreaPoint45 }; rayCasterPoint45.generateRays(rng, 1000, true);
				Bvh bvhPoint45{ bvhProperties, influenceAreaPoint45, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPoint45) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Random1000_Point_45_Longest", random1000Triangles, vector{&rayCasterPoint45}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Random1000_Point_45_Longest", scene.buildAndTraverse());
			}

			// === Random1000 scene: 1 point influence area oblique covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPointOblique{ Pov{{10,12,-3}, {-.55,-.9,1}, 60, 60}, 20, 1, 10000 };
				PointRayCaster rayCasterPointOblique{ influenceAreaPointOblique }; rayCasterPointOblique.generateRays(rng, 1000, true);
				Bvh bvhPointOblique{ bvhProperties, influenceAreaPointOblique, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPointOblique) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Random1000_Point_Oblique_Longest", random1000Triangles, vector{&rayCasterPointOblique}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Random1000_Point_Oblique_Longest", scene.buildAndTraverse());
			}
		}

		// === Sponza scene: 1 point influence area ===
		if constexpr (POINT_LONGEST and_or SPONZA_SCENE) {
			// === Sponza scene: 1 point influence area parallel to x covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPointParallel{ Pov{{-9,11,-1}, {1,0,0}, 70, 50}, 50, 1, 10000 };
				PointRayCaster rayCasterPointParallel{ influenceAreaPointParallel }; rayCasterPointParallel.generateRays(rng, 1000, true);
				Bvh bvhPointParallel{ bvhProperties, influenceAreaPointParallel, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPointParallel) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Sponza_Point_Parallel_Longest", sponzaTriangles, vector{&rayCasterPointParallel}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Sponza_Point_Parallel_Longest", scene.buildAndTraverse());
			}

			// === Sponza scene: 1 point influence area 15 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPoint15{ Pov{{-12,11,0.5}, {.966,0,-.259}, 70,50}, 60,1, 10000 };
				PointRayCaster rayCasterPoint15{ influenceAreaPoint15 }; rayCasterPoint15.generateRays(rng, 1000, true);
				Bvh bvhPoint15{ bvhProperties, influenceAreaPoint15, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPoint15) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Sponza_Point_15_Longest", sponzaTriangles, vector{&rayCasterPoint15}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Sponza_Point_15_Longest", scene.buildAndTraverse());
			}

			// === Sponza scene: 1 point influence area 45 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPoint45{ Pov{{-10,11,18}, {1,0,-1}, 90,50}, 70,1, 10000 };
				PointRayCaster rayCasterPoint45{ influenceAreaPoint45 }; rayCasterPoint45.generateRays(rng, 1000, true);
				Bvh bvhPoint45{ bvhProperties, influenceAreaPoint45, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPoint45) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Sponza_Point_45_Longest", sponzaTriangles, vector{&rayCasterPoint45}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Sponza_Point_45_Longest", scene.buildAndTraverse());
			}

			// === Sponza scene: 1 point influence area oblique covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPointOblique{ Pov{{-12,10,7.2}, {.84,0.26,-.48}, 70,50}, 70,1, 10000 };
				PointRayCaster rayCasterPointOblique{ influenceAreaPointOblique }; rayCasterPointOblique.generateRays(rng, 1000, true);
				Bvh bvhPointOblique{ bvhProperties, influenceAreaPointOblique, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPointOblique) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Sponza_Point_Oblique_Longest", sponzaTriangles, vector{&rayCasterPointOblique}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Sponza_Point_Oblique_Longest", scene.buildAndTraverse());
			}
		}
	};

	// PLANE SAH+SPFH
	bvhProperties.splitPlaneQualityThreshold = spfhSplitPlaneQualityThreshold;
	{
		// === Wood scene: 1 plane influence area ===
		if constexpr (PLANE_FACING and_or WOOD_SCENE) {
			// === Wood scene: 1 plane influence area parallel to x covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlaneParallel{ Plane{{2.5,.9,0}, {-1,0,0}, 1.5,0.9}, 10, 10000 };
				PlaneRayCaster rayCasterPlaneParallel{ influenceAreaPlaneParallel }; rayCasterPlaneParallel.generateRays(rng, 1000, true);
				Bvh bvhPlaneParallel{ bvhProperties, influenceAreaPlaneParallel, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlaneParallel) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Wood_Plane_Parallel_Facing", woodTriangles, vector{&rayCasterPlaneParallel}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Wood_Plane_Parallel_Facing", scene.buildAndTraverse());
			}

			// === Wood scene: 1 plane influence area 15 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlane15{ Plane{{2.5,.8,-.3}, {-.966,0,.259}, 1.6,1}, 10, 10000 };
				PlaneRayCaster rayCasterPlane15{ influenceAreaPlane15 }; rayCasterPlane15.generateRays(rng, 1000, true);
				Bvh bvhPlane15{ bvhProperties, influenceAreaPlane15, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlane15) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Wood_Plane_15_Facing", woodTriangles, vector{&rayCasterPlane15}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Wood_Plane_15_Facing", scene.buildAndTraverse());
			}

			// === Wood scene: 1 plane influence area 45 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlane45{ Plane{{2.5,.8,-3}, {-1,0,1}, 1.5,1}, 10, 10000 };
				PlaneRayCaster rayCasterPlane45{ influenceAreaPlane45 }; rayCasterPlane45.generateRays(rng, 1000, true);
				Bvh bvhPlane45{ bvhProperties, influenceAreaPlane45, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlane45) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Wood_Plane_45_Facing", woodTriangles, vector{&rayCasterPlane45}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Wood_Plane_45_Facing", scene.buildAndTraverse());
			}

			// === Wood scene: 1 plane influence area oblique covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlaneOblique{ Plane{{2.5,1.5,-1}, {-1,-0.3,0.4}, 1.8,1.3}, 20, 10000 };
				PlaneRayCaster rayCasterPlaneOblique{ influenceAreaPlaneOblique }; rayCasterPlaneOblique.generateRays(rng, 1000, true);
				Bvh bvhPlaneOblique{ bvhProperties, influenceAreaPlaneOblique, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlaneOblique) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Wood_Plane_Oblique_Facing", woodTriangles, vector{&rayCasterPlaneOblique}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Wood_Plane_Oblique_Facing", scene.buildAndTraverse());
			}
		}

		// === Suzanne scene: 1 plane influence area ===
		if constexpr (PLANE_FACING and_or SUZANNE_SCENE) {
			// === Suzanne scene: 1 plane influence area parallel to x covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlaneParallel{ Plane{{0,0,2}, {0,0,-1}, .9,.9}, 10, 10000 };
				PlaneRayCaster rayCasterPlaneParallel{ influenceAreaPlaneParallel }; rayCasterPlaneParallel.generateRays(rng, 1000, true);
				Bvh bvhPlaneParallel{ bvhProperties, influenceAreaPlaneParallel, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlaneParallel) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "SuzannePlaneParallel_Facing", suzanneTriangles, vector{&rayCasterPlaneParallel}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("SuzannePlaneParallel_Facing", scene.buildAndTraverse());
			}

			// === Suzanne scene: 1 plane influence area 15 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlane15{ Plane{{.5,0,2}, {-.259,0,-.966}, 1,.9}, 10, 10000 };
				PlaneRayCaster rayCasterPlane15{ influenceAreaPlane15 }; rayCasterPlane15.generateRays(rng, 1000, true);
				Bvh bvhPlane15{ bvhProperties, influenceAreaPlane15, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlane15) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "SuzannePlane15_Facing", suzanneTriangles, vector{&rayCasterPlane15}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("SuzannePlane15_Facing", scene.buildAndTraverse());
			}

			// === Suzanne scene: 1 plane influence area 45 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlane45{ Plane{{2,0,2}, {-1,0,-1}, 1.2,.9}, 10, 10000 };
				PlaneRayCaster rayCasterPlane45{ influenceAreaPlane45 }; rayCasterPlane45.generateRays(rng, 1000, true);
				Bvh bvhPlane45{ bvhProperties, influenceAreaPlane45, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlane45) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "SuzannePlane45_Facing", suzanneTriangles, vector{&rayCasterPlane45}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("SuzannePlane45_Facing", scene.buildAndTraverse());
			}

			// === Suzanne scene: 1 plane influence area oblique covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlaneOblique{ Plane{{1,-.5,2}, {-.45,.3,-1}, 2,1.5}, 10, 10000 };
				PlaneRayCaster rayCasterPlaneOblique{ influenceAreaPlaneOblique }; rayCasterPlaneOblique.generateRays(rng, 1000, true);
				Bvh bvhPlaneOblique{ bvhProperties, influenceAreaPlaneOblique, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlaneOblique) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "SuzannePlaneOblique_Facing", suzanneTriangles, vector{&rayCasterPlaneOblique}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("SuzannePlaneOblique_Facing", scene.buildAndTraverse());
			}
		}

		// === Cottage scene: 1 plane influence area ===
		if constexpr (PLANE_FACING and_or COTTAGE_SCENE) {
			// === Cottage scene: 1 plane influence area parallel to x covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlaneParallel{ Plane{{10,2.8,0}, {-1,0,0}, 6,2.6}, 35, 10000 };
				PlaneRayCaster rayCasterPlaneParallel{ influenceAreaPlaneParallel }; rayCasterPlaneParallel.generateRays(rng, 1000, true);
				Bvh bvhPlaneParallel{ bvhProperties, influenceAreaPlaneParallel, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlaneParallel) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Cottage_Plane_Parallel_Facing", cottageTriangles, vector{&rayCasterPlaneParallel}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Cottage_Plane_Parallel_Facing", scene.buildAndTraverse());
			}

			// === Cottage scene: 1 plane influence area 15 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlane15{ Plane{{10,2.9,-3}, {-.966,0,.259}, 6,2.9}, 35, 10000 };
				PlaneRayCaster rayCasterPlane15{ influenceAreaPlane15 }; rayCasterPlane15.generateRays(rng, 1000, true);
				Bvh bvhPlane15{ bvhProperties, influenceAreaPlane15, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlane15) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Cottage_Plane_15_Facing", cottageTriangles, vector{&rayCasterPlane15}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Cottage_Plane_15_Facing", scene.buildAndTraverse());
			}

			// === Cottage scene: 1 plane influence area 45 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlane45{ Plane{{7.5,2.9,-7.5}, {-1,0,1}, 8,2.9}, 35, 10000 };
				PlaneRayCaster rayCasterPlane45{ influenceAreaPlane45 }; rayCasterPlane45.generateRays(rng, 1000, true);
				Bvh bvhPlane45{ bvhProperties, influenceAreaPlane45, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlane45) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Cottage_Plane_45_Facing", cottageTriangles, vector{&rayCasterPlane45}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Cottage_Plane_45_Facing", scene.buildAndTraverse());
			}

			// === Cottage scene: 1 plane influence area oblique covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlaneOblique{ Plane{{10,5,-8}, {-1,-.3,.9}, 7,3.3}, 35, 10000 };
				PlaneRayCaster rayCasterPlaneOblique{ influenceAreaPlaneOblique }; rayCasterPlaneOblique.generateRays(rng, 1000, true);
				Bvh bvhPlaneOblique{ bvhProperties, influenceAreaPlaneOblique, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlaneOblique) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Cottage_Plane_Oblique_Facing", cottageTriangles, vector{&rayCasterPlaneOblique}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Cottage_Plane_Oblique_Facing", scene.buildAndTraverse());
			}
		}

		// === CottageWalls scene: 1 plane influence area ===
		if constexpr (PLANE_FACING and_or COTTAGE_WALLS_SCENE) {
			// === CottageWalls scene: 1 plane influence area parallel to x covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlaneParallel{ Plane{{10,2.8,0}, {-1,0,0}, 6,2.6}, 35, 10000 };
				PlaneRayCaster rayCasterPlaneParallel{ influenceAreaPlaneParallel }; rayCasterPlaneParallel.generateRays(rng, 1000, true);
				Bvh bvhPlaneParallel{ bvhProperties, influenceAreaPlaneParallel, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlaneParallel) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "CottageWalls_Plane_Parallel_Facing", cottageWallsTriangles, vector{&rayCasterPlaneParallel}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("CottageWalls_Plane_Parallel_Facing", scene.buildAndTraverse());
			}

			// === CottageWalls scene: 1 plane influence area 15 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlane15{ Plane{{10,2.9,-3}, {-.966,0,.259}, 6,2.9}, 35, 10000 };
				PlaneRayCaster rayCasterPlane15{ influenceAreaPlane15 }; rayCasterPlane15.generateRays(rng, 1000, true);
				Bvh bvhPlane15{ bvhProperties, influenceAreaPlane15, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlane15) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "CottageWalls_Plane_15_Facing", cottageWallsTriangles, vector{&rayCasterPlane15}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("CottageWalls_Plane_15_Facing", scene.buildAndTraverse());
			}

			// === CottageWalls scene: 1 plane influence area 45 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlane45{ Plane{{7.5,2.9,-7.5}, {-1,0,1}, 8,2.9}, 35, 10000 };
				PlaneRayCaster rayCasterPlane45{ influenceAreaPlane45 }; rayCasterPlane45.generateRays(rng, 1000, true);
				Bvh bvhPlane45{ bvhProperties, influenceAreaPlane45, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlane45) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "CottageWalls_Plane_45_Facing", cottageWallsTriangles, vector{&rayCasterPlane45}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("CottageWalls_Plane_45_Facing", scene.buildAndTraverse());
			}

			// === CottageWalls scene: 1 plane influence area oblique covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlaneOblique{ Plane{{10,5,-8}, {-1,-.3,.9}, 7,3.3}, 35, 10000 };
				PlaneRayCaster rayCasterPlaneOblique{ influenceAreaPlaneOblique }; rayCasterPlaneOblique.generateRays(rng, 1000, true);
				Bvh bvhPlaneOblique{ bvhProperties, influenceAreaPlaneOblique, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlaneOblique) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "CottageWalls_Plane_Oblique_Facing", cottageWallsTriangles, vector{&rayCasterPlaneOblique}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("CottageWalls_Plane_Oblique_Facing", scene.buildAndTraverse());
			}
		}

		// === Crowd scene: 1 plane influence area ===
		if constexpr (PLANE_FACING and_or CROWD_SCENE) {
			// === Crowd scene: 1 plane influence area parallel to x covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlaneParallel{ Plane{{2,2.2,-8}, {0,0,1}, 7,1.8}, 30, 10000 };
				PlaneRayCaster rayCasterPlaneParallel{ influenceAreaPlaneParallel }; rayCasterPlaneParallel.generateRays(rng, 1000, true);
				Bvh bvhPlaneParallel{ bvhProperties, influenceAreaPlaneParallel, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlaneParallel) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Crowd_Plane_Parallel_Facing", crowdTriangles, vector{&rayCasterPlaneParallel}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Crowd_Plane_Parallel_Facing", scene.buildAndTraverse());
			}

			// === Crowd scene: 1 plane influence area 15 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlane15{ Plane{{-2.5,2.2,-8}, {.259,0,.966}, 7.5,1.8}, 30, 10000 };
				PlaneRayCaster rayCasterPlane15{ influenceAreaPlane15 }; rayCasterPlane15.generateRays(rng, 1000, true);
				Bvh bvhPlane15{ bvhProperties, influenceAreaPlane15, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlane15) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Crowd_Plane_15_Facing", crowdTriangles, vector{&rayCasterPlane15}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Crowd_Plane_15_Facing", scene.buildAndTraverse());
			}

			// === Crowd scene: 1 plane influence area 45 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlane45{ Plane{{-8,2.2,-8}, {1,0,1}, 9,1.8}, 30, 10000 };
				PlaneRayCaster rayCasterPlane45{ influenceAreaPlane45 }; rayCasterPlane45.generateRays(rng, 1000, true);
				Bvh bvhPlane45{ bvhProperties, influenceAreaPlane45, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlane45) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Crowd_Plane_45_Facing", crowdTriangles, vector{&rayCasterPlane45}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Crowd_Plane_45_Facing", scene.buildAndTraverse());
			}

			// === Crowd scene: 1 plane influence area oblique covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlaneOblique{ Plane{{-6,7,-8}, {.6,-.4,1}, 7.5,3.4}, 35, 10000 };
				PlaneRayCaster rayCasterPlaneOblique{ influenceAreaPlaneOblique }; rayCasterPlaneOblique.generateRays(rng, 1000, true);
				Bvh bvhPlaneOblique{ bvhProperties, influenceAreaPlaneOblique, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlaneOblique) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Crowd_Plane_Oblique_Facing", crowdTriangles, vector{&rayCasterPlaneOblique}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Crowd_Plane_Oblique_Facing", scene.buildAndTraverse());
			}
		}

		// === Random100 scene: 1 plane influence area ===
		if constexpr (PLANE_FACING and_or RANDOM100_SCENE) {
			// === Random100 scene: 1 plane influence area parallel to x covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlaneParallel{ Plane{{5,5,-4}, {0,0,1}, 5,5}, 20, 10000 };
				PlaneRayCaster rayCasterPlaneParallel{ influenceAreaPlaneParallel }; rayCasterPlaneParallel.generateRays(rng, 1000, true);
				Bvh bvhPlaneParallel{ bvhProperties, influenceAreaPlaneParallel, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlaneParallel) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Random100_Plane_Parallel_Facing", random100Triangles, vector{&rayCasterPlaneParallel}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Random100_Plane_Parallel_Facing", scene.buildAndTraverse());
			}

			// === Random100 scene: 1 plane influence area 15 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlane15{ Plane{{3.5,5,-4}, {.259,0,.966}, 6.5,5}, 20, 10000 };
				PlaneRayCaster rayCasterPlane15{ influenceAreaPlane15 }; rayCasterPlane15.generateRays(rng, 1000, true);
				Bvh bvhPlane15{ bvhProperties, influenceAreaPlane15, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlane15) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Random100_Plane_15_Facing", random100Triangles, vector{&rayCasterPlane15}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Random100_Plane_15_Facing", scene.buildAndTraverse());
			}

			// === Random100 scene: 1 plane influence area 45 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlane45{ Plane{{-3,5,-3}, {1,0,1}, 7,5}, 20, 10000 };
				PlaneRayCaster rayCasterPlane45{ influenceAreaPlane45 }; rayCasterPlane45.generateRays(rng, 1000, true);
				Bvh bvhPlane45{ bvhProperties, influenceAreaPlane45, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlane45) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Random100_Plane_45_Facing", random100Triangles, vector{&rayCasterPlane45}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Random100_Plane_45_Facing", scene.buildAndTraverse());
			}

			// === Random100 scene: 1 plane influence area oblique covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlaneOblique{ Plane{{10,12,-3}, {-.55,-.9,1}, 6.5,6.5}, 20, 10000 };
				PlaneRayCaster rayCasterPlaneOblique{ influenceAreaPlaneOblique }; rayCasterPlaneOblique.generateRays(rng, 1000, true);
				Bvh bvhPlaneOblique{ bvhProperties, influenceAreaPlaneOblique, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlaneOblique) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Random100_Plane_Oblique_Facing", random100Triangles, vector{&rayCasterPlaneOblique}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Random100_Plane_Oblique_Facing", scene.buildAndTraverse());
			}
		}

		// === Random1000 scene: 1 plane influence area ===
		if constexpr (PLANE_FACING and_or RANDOM1000_SCENE) {
			// === Random1000 scene: 1 plane influence area parallel to x covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlaneParallel{ Plane{{5,5,-4}, {0,0,1}, 5,5}, 20, 10000 };
				PlaneRayCaster rayCasterPlaneParallel{ influenceAreaPlaneParallel }; rayCasterPlaneParallel.generateRays(rng, 1000, true);
				Bvh bvhPlaneParallel{ bvhProperties, influenceAreaPlaneParallel, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlaneParallel) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Random1000_Plane_Parallel_Facing", random1000Triangles, vector{&rayCasterPlaneParallel}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Random1000_Plane_Parallel_Facing", scene.buildAndTraverse());
			}

			// === Random1000 scene: 1 plane influence area 15 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlane15{ Plane{{3.5,5,-4}, {.259,0,.966}, 6.5,5}, 20, 10000 };
				PlaneRayCaster rayCasterPlane15{ influenceAreaPlane15 }; rayCasterPlane15.generateRays(rng, 1000, true);
				Bvh bvhPlane15{ bvhProperties, influenceAreaPlane15, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlane15) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Random1000_Plane_15_Facing", random1000Triangles, vector{&rayCasterPlane15}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Random1000_Plane_15_Facing", scene.buildAndTraverse());
			}

			// === Random1000 scene: 1 plane influence area 45 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlane45{ Plane{{-3,5,-3}, {1,0,1}, 7,5}, 20, 10000 };
				PlaneRayCaster rayCasterPlane45{ influenceAreaPlane45 }; rayCasterPlane45.generateRays(rng, 1000, true);
				Bvh bvhPlane45{ bvhProperties, influenceAreaPlane45, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlane45) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Random1000_Plane_45_Facing", random1000Triangles, vector{&rayCasterPlane45}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Random1000_Plane_45_Facing", scene.buildAndTraverse());
			}

			// === Random1000 scene: 1 plane influence area oblique covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlaneOblique{ Plane{{10,12,-3}, {-.55,-.9,1}, 6.5,6.5}, 20, 10000 };
				PlaneRayCaster rayCasterPlaneOblique{ influenceAreaPlaneOblique }; rayCasterPlaneOblique.generateRays(rng, 1000, true);
				Bvh bvhPlaneOblique{ bvhProperties, influenceAreaPlaneOblique, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlaneOblique) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Random1000_Plane_Oblique_Facing", random1000Triangles, vector{&rayCasterPlaneOblique}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Random1000_Plane_Oblique_Facing", scene.buildAndTraverse());
			}
		}

		// === Sponza scene: 1 plane influence area ===
		if constexpr (PLANE_FACING and_or SPONZA_SCENE) {
			// === Sponza scene: 1 plane influence area parallel to x covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlaneParallel{ Plane{{-9,11,-1}, {1,0,0}, 16, 11}, 50, 10000 };
				PlaneRayCaster rayCasterPlaneParallel{ influenceAreaPlaneParallel }; rayCasterPlaneParallel.generateRays(rng, 1000, true);
				Bvh bvhPlaneParallel{ bvhProperties, influenceAreaPlaneParallel, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlaneParallel) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Sponza_Plane_Parallel_Facing", sponzaTriangles, vector{&rayCasterPlaneParallel}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Sponza_Plane_Parallel_Facing", scene.buildAndTraverse());
			}

			// === Sponza scene: 1 plane influence area 15 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlane15{ Plane{{-12,11,0.5}, {.966,0,-.259}, 16,11}, 60, 10000 };
				PlaneRayCaster rayCasterPlane15{ influenceAreaPlane15 }; rayCasterPlane15.generateRays(rng, 1000, true);
				Bvh bvhPlane15{ bvhProperties, influenceAreaPlane15, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlane15) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Sponza_Plane_15_Facing", sponzaTriangles, vector{&rayCasterPlane15}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Sponza_Plane_15_Facing", scene.buildAndTraverse());
			}

			// === Sponza scene: 1 plane influence area 45 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlane45{ Plane{{-10,11,18}, {1,0,-1}, 25,11}, 70, 10000 };
				PlaneRayCaster rayCasterPlane45{ influenceAreaPlane45 }; rayCasterPlane45.generateRays(rng, 1000, true);
				Bvh bvhPlane45{ bvhProperties, influenceAreaPlane45, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlane45) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Sponza_Plane_45_Facing", sponzaTriangles, vector{&rayCasterPlane45}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Sponza_Plane_45_Facing", scene.buildAndTraverse());
			}

			// === Sponza scene: 1 plane influence area oblique covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlaneOblique{ Plane{{-12,10,7.2}, {.84,0.26,-.48}, 16,11}, 70, 10000 };
				PlaneRayCaster rayCasterPlaneOblique{ influenceAreaPlaneOblique }; rayCasterPlaneOblique.generateRays(rng, 1000, true);
				Bvh bvhPlaneOblique{ bvhProperties, influenceAreaPlaneOblique, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlaneOblique) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Sponza_Plane_Oblique_Facing", sponzaTriangles, vector{&rayCasterPlaneOblique}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Sponza_Plane_Oblique_Facing", scene.buildAndTraverse());
			}
		}
	};

	// POINT SAH+SPFH
	bvhProperties.splitPlaneQualityThreshold = spfhSplitPlaneQualityThreshold;
	{
		// === Wood scene: 1 point influence area ===
		if constexpr (POINT_FACING and_or WOOD_SCENE) {
			// === Wood scene: 1 point influence area parallel to x covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPointParallel{ Pov{{2.5,.9,0}, {-1,0,0}, 90, 60}, 10, 1, 10000 };
				PointRayCaster rayCasterPointParallel{ influenceAreaPointParallel }; rayCasterPointParallel.generateRays(rng, 1000, true);
				Bvh bvhPointParallel{ bvhProperties, influenceAreaPointParallel, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPointParallel) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Wood_Point_Parallel_Facing", woodTriangles, vector{&rayCasterPointParallel}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Wood_Point_Parallel_Facing", scene.buildAndTraverse());
			}

			// === Wood scene: 1 point influence area 15 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPoint15{ Pov{{2.5,.8,-.3}, {-.966,0,.259}, 90, 60}, 10, 1, 10000 };
				PointRayCaster rayCasterPoint15{ influenceAreaPoint15 }; rayCasterPoint15.generateRays(rng, 1000, true);
				Bvh bvhPoint15{ bvhProperties, influenceAreaPoint15, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPoint15) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Wood_Point_15_Facing", woodTriangles, vector{&rayCasterPoint15}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Wood_Point_15_Facing", scene.buildAndTraverse());
			}

			// === Wood scene: 1 point influence area 45 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPoint45{ Pov{{2.5,.8,-3}, {-1,0,1}, 90, 60}, 10, 1, 10000 };
				PointRayCaster rayCasterPoint45{ influenceAreaPoint45 }; rayCasterPoint45.generateRays(rng, 1000, true);
				Bvh bvhPoint45{ bvhProperties, influenceAreaPoint45, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPoint45) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Wood_Point_45_Facing", woodTriangles, vector{&rayCasterPoint45}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Wood_Point_45_Facing", scene.buildAndTraverse());
			}

			// === Wood scene: 1 point influence area oblique covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPointOblique{ Pov{{2.5,1.5,-1}, {-1,-0.3,0.4}, 90, 60}, 20, 1, 10000 };
				PointRayCaster rayCasterPointOblique{ influenceAreaPointOblique }; rayCasterPointOblique.generateRays(rng, 1000, true);
				Bvh bvhPointOblique{ bvhProperties, influenceAreaPointOblique, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPointOblique) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Wood_Point_Oblique_Facing", woodTriangles, vector{&rayCasterPointOblique}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Wood_Point_Oblique_Facing", scene.buildAndTraverse());
			}
		}

		// === Suzanne scene: 1 point influence area ===
		if constexpr (POINT_FACING and_or SUZANNE_SCENE) {
			// === Suzanne scene: 1 point influence area parallel to x covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPointParallel{ Pov{{0,0,4}, {0,0,-1}, 30, 30}, 10, 2, 10000 };
				PointRayCaster rayCasterPointParallel{ influenceAreaPointParallel }; rayCasterPointParallel.generateRays(rng, 1000, true);
				Bvh bvhPointParallel{ bvhProperties, influenceAreaPointParallel, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPointParallel) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "SuzannePointParallel_Facing", suzanneTriangles, vector{&rayCasterPointParallel}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("SuzannePointParallel_Facing", scene.buildAndTraverse());
			}

			// === Suzanne scene: 1 point influence area 15 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPoint15{ Pov{{1.3,0,5}, {-.259,0,-.966}, 25, 25}, 10, 2, 10000 };
				PointRayCaster rayCasterPoint15{ influenceAreaPoint15 }; rayCasterPoint15.generateRays(rng, 1000, true);
				Bvh bvhPoint15{ bvhProperties, influenceAreaPoint15, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPoint15) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "SuzannePoint15_Facing", suzanneTriangles, vector{&rayCasterPoint15}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("SuzannePoint15_Facing", scene.buildAndTraverse());
			}

			// === Suzanne scene: 1 point influence area 45 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPoint45{ Pov{{3.3,0,3}, {-1,0,-1}, 30, 30}, 10, 2, 10000 };
				PointRayCaster rayCasterPoint45{ influenceAreaPoint45 }; rayCasterPoint45.generateRays(rng, 1000, true);
				Bvh bvhPoint45{ bvhProperties, influenceAreaPoint45, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPoint45) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "SuzannePoint45_Facing", suzanneTriangles, vector{&rayCasterPoint45}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("SuzannePoint45_Facing", scene.buildAndTraverse());
			}

			// === Suzanne scene: 1 point influence area oblique covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPointOblique{ Pov{{2,-1.2,4}, {-.45,.3,-1}, 30, 30}, 10, 2, 10000 };
				PointRayCaster rayCasterPointOblique{ influenceAreaPointOblique }; rayCasterPointOblique.generateRays(rng, 1000, true);
				Bvh bvhPointOblique{ bvhProperties, influenceAreaPointOblique, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPointOblique) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "SuzannePointOblique_Facing", suzanneTriangles, vector{&rayCasterPointOblique}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("SuzannePointOblique_Facing", scene.buildAndTraverse());
			}
		}

		// === Cottage scene: 1 point influence area ===
		if constexpr (POINT_FACING and_or COTTAGE_SCENE) {
			// === Cottage scene: 1 point influence area parallel to x covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPointParallel{ Pov{{15,2.7,0}, {-1,0,0}, 70, 35}, 35, 1, 10000 };
				PointRayCaster rayCasterPointParallel{ influenceAreaPointParallel }; rayCasterPointParallel.generateRays(rng, 1000, true);
				Bvh bvhPointParallel{ bvhProperties, influenceAreaPointParallel, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPointParallel) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Cottage_Point_Parallel_Facing", cottageTriangles, vector{&rayCasterPointParallel}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Cottage_Point_Parallel_Facing", scene.buildAndTraverse());
			}

			// === Cottage scene: 1 point influence area 15 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPoint15{ Pov{{15,2.7,-3}, {-.966,0,.259}, 70, 35}, 35, 1, 10000 };
				PointRayCaster rayCasterPoint15{ influenceAreaPoint15 }; rayCasterPoint15.generateRays(rng, 1000, true);
				Bvh bvhPoint15{ bvhProperties, influenceAreaPoint15, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPoint15) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Cottage_Point_15_Facing", cottageTriangles, vector{&rayCasterPoint15}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Cottage_Point_15_Facing", scene.buildAndTraverse());
			}

			// === Cottage scene: 1 point influence area 45 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPoint45{ Pov{{13,2.5,-13}, {-1,0,1}, 50, 20}, 35, 1, 10000 };
				PointRayCaster rayCasterPoint45{ influenceAreaPoint45 }; rayCasterPoint45.generateRays(rng, 1000, true);
				Bvh bvhPoint45{ bvhProperties, influenceAreaPoint45, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPoint45) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Cottage_Point_45_Facing", cottageTriangles, vector{&rayCasterPoint45}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Cottage_Point_45_Facing", scene.buildAndTraverse());
			}

			// === Cottage scene: 1 point influence area oblique covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPointOblique{ Pov{{15,7,-11}, {-1,-.45,.75}, 50, 30}, 35, 1, 10000 };
				PointRayCaster rayCasterPointOblique{ influenceAreaPointOblique }; rayCasterPointOblique.generateRays(rng, 1000, true);
				Bvh bvhPointOblique{ bvhProperties, influenceAreaPointOblique, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPointOblique) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Cottage_Point_Oblique_Facing", cottageTriangles, vector{&rayCasterPointOblique}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Cottage_Point_Oblique_Facing", scene.buildAndTraverse());
			}
		}

		// === CottageWalls scene: 1 point influence area ===
		if constexpr (POINT_FACING and_or COTTAGE_WALLS_SCENE) {
			// === CottageWalls scene: 1 point influence area parallel to x covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPointParallel{ Pov{{15,2.7,0}, {-1,0,0}, 70, 35}, 35, 1, 10000 };
				PointRayCaster rayCasterPointParallel{ influenceAreaPointParallel }; rayCasterPointParallel.generateRays(rng, 1000, true);
				Bvh bvhPointParallel{ bvhProperties, influenceAreaPointParallel, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPointParallel) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "CottageWalls_Point_Parallel_Facing", cottageWallsTriangles, vector{&rayCasterPointParallel}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("CottageWalls_Point_Parallel_Facing", scene.buildAndTraverse());
			}

			// === CottageWalls scene: 1 point influence area 15 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPoint15{ Pov{{15,2.7,-3}, {-.966,0,.259}, 70, 35}, 35, 1, 10000 };
				PointRayCaster rayCasterPoint15{ influenceAreaPoint15 }; rayCasterPoint15.generateRays(rng, 1000, true);
				Bvh bvhPoint15{ bvhProperties, influenceAreaPoint15, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPoint15) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "CottageWalls_Point_15_Facing", cottageWallsTriangles, vector{&rayCasterPoint15}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("CottageWalls_Point_15_Facing", scene.buildAndTraverse());
			}

			// === CottageWalls scene: 1 point influence area 45 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPoint45{ Pov{{13,2.5,-13}, {-1,0,1}, 50, 20}, 35, 1, 10000 };
				PointRayCaster rayCasterPoint45{ influenceAreaPoint45 }; rayCasterPoint45.generateRays(rng, 1000, true);
				Bvh bvhPoint45{ bvhProperties, influenceAreaPoint45, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPoint45) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "CottageWalls_Point_45_Facing", cottageWallsTriangles, vector{&rayCasterPoint45}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("CottageWalls_Point_45_Facing", scene.buildAndTraverse());
			}

			// === CottageWalls scene: 1 point influence area oblique covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPointOblique{ Pov{{15,7,-11}, {-1,-.45,.75}, 50, 30}, 35, 1, 10000 };
				PointRayCaster rayCasterPointOblique{ influenceAreaPointOblique }; rayCasterPointOblique.generateRays(rng, 1000, true);
				Bvh bvhPointOblique{ bvhProperties, influenceAreaPointOblique, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPointOblique) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "CottageWalls_Point_Oblique_Facing", cottageWallsTriangles, vector{&rayCasterPointOblique}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("CottageWalls_Point_Oblique_Facing", scene.buildAndTraverse());
			}
		}

		// === Crowd scene: 1 point influence area ===
		if constexpr (POINT_FACING and_or CROWD_SCENE) {
			// === Crowd scene: 1 point influence area parallel to x covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPointParallel{ Pov{{2,2.2,-8}, {0,0,1}, 130, 45}, 30, 1, 10000 };
				PointRayCaster rayCasterPointParallel{ influenceAreaPointParallel }; rayCasterPointParallel.generateRays(rng, 1000, true);
				Bvh bvhPointParallel{ bvhProperties, influenceAreaPointParallel, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPointParallel) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Crowd_Point_Parallel_Facing", crowdTriangles, vector{&rayCasterPointParallel}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Crowd_Point_Parallel_Facing", scene.buildAndTraverse());
			}

			// === Crowd scene: 1 point influence area 15 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPoint15{ Pov{{-2.5,2.2,-8}, {.259,0,.966}, 130, 45}, 30, 1, 10000 };
				PointRayCaster rayCasterPoint15{ influenceAreaPoint15 }; rayCasterPoint15.generateRays(rng, 1000, true);
				Bvh bvhPoint15{ bvhProperties, influenceAreaPoint15, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPoint15) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Crowd_Point_15_Facing", crowdTriangles, vector{&rayCasterPoint15}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Crowd_Point_15_Facing", scene.buildAndTraverse());
			}

			// === Crowd scene: 1 point influence area 45 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPoint45{ Pov{{-8,2.2,-8}, {1,0,1}, 130, 45}, 30, 1, 10000 };
				PointRayCaster rayCasterPoint45{ influenceAreaPoint45 }; rayCasterPoint45.generateRays(rng, 1000, true);
				Bvh bvhPoint45{ bvhProperties, influenceAreaPoint45, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPoint45) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Crowd_Point_45_Facing", crowdTriangles, vector{&rayCasterPoint45}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Crowd_Point_45_Facing", scene.buildAndTraverse());
			}

			// === Crowd scene: 1 point influence area oblique covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPointOblique{ Pov{{-6,7,-8}, {.6,-.4,1}, 130, 45}, 35, 1, 10000 };
				PointRayCaster rayCasterPointOblique{ influenceAreaPointOblique }; rayCasterPointOblique.generateRays(rng, 1000, true);
				Bvh bvhPointOblique{ bvhProperties, influenceAreaPointOblique, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPointOblique) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Crowd_Point_Oblique_Facing", crowdTriangles, vector{&rayCasterPointOblique}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Crowd_Point_Oblique_Facing", scene.buildAndTraverse());
			}
		}

		// === Random100 scene: 1 point influence area ===
		if constexpr (POINT_FACING and_or RANDOM100_SCENE) {
			// === Random100 scene: 1 point influence area parallel to x covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPointParallel{ Pov{{5,5,-4}, {0,0,1}, 90, 90}, 20, 1, 10000 };
				PointRayCaster rayCasterPointParallel{ influenceAreaPointParallel }; rayCasterPointParallel.generateRays(rng, 1000, true);
				Bvh bvhPointParallel{ bvhProperties, influenceAreaPointParallel, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPointParallel) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Random100_Point_Parallel_Facing", random100Triangles, vector{&rayCasterPointParallel}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Random100_Point_Parallel_Facing", scene.buildAndTraverse());
			}

			// === Random100 scene: 1 point influence area 15 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPoint15{ Pov{{3.5,5,-4}, {.259,0,.966}, 90, 90}, 20, 1, 10000 };
				PointRayCaster rayCasterPoint15{ influenceAreaPoint15 }; rayCasterPoint15.generateRays(rng, 1000, true);
				Bvh bvhPoint15{ bvhProperties, influenceAreaPoint15, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPoint15) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Random100_Point_15_Facing", random100Triangles, vector{&rayCasterPoint15}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Random100_Point_15_Facing", scene.buildAndTraverse());
			}

			// === Random100 scene: 1 point influence area 45 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPoint45{ Pov{{-3,5,-3}, {1,0,1}, 60, 90}, 20, 1, 10000 };
				PointRayCaster rayCasterPoint45{ influenceAreaPoint45 }; rayCasterPoint45.generateRays(rng, 1000, true);
				Bvh bvhPoint45{ bvhProperties, influenceAreaPoint45, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPoint45) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Random100_Point_45_Facing", random100Triangles, vector{&rayCasterPoint45}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Random100_Point_45_Facing", scene.buildAndTraverse());
			}

			// === Random100 scene: 1 point influence area oblique covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPointOblique{ Pov{{10,12,-3}, {-.55,-.9,1}, 60, 60}, 20, 1, 10000 };
				PointRayCaster rayCasterPointOblique{ influenceAreaPointOblique }; rayCasterPointOblique.generateRays(rng, 1000, true);
				Bvh bvhPointOblique{ bvhProperties, influenceAreaPointOblique, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPointOblique) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Random100_Point_Oblique_Facing", random100Triangles, vector{&rayCasterPointOblique}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Random100_Point_Oblique_Facing", scene.buildAndTraverse());
			}
		}

		// === Random1000 scene: 1 point influence area ===
		if constexpr (POINT_FACING and_or RANDOM1000_SCENE) {
			// === Random1000 scene: 1 point influence area parallel to x covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPointParallel{ Pov{{5,5,-4}, {0,0,1}, 90, 90}, 20, 1, 10000 };
				PointRayCaster rayCasterPointParallel{ influenceAreaPointParallel }; rayCasterPointParallel.generateRays(rng, 1000, true);
				Bvh bvhPointParallel{ bvhProperties, influenceAreaPointParallel, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPointParallel) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Random1000_Point_Parallel_Facing", random1000Triangles, vector{&rayCasterPointParallel}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Random1000_Point_Parallel_Facing", scene.buildAndTraverse());
			}

			// === Random1000 scene: 1 point influence area 15 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPoint15{ Pov{{3.5,5,-4}, {.259,0,.966}, 90, 90}, 20, 1, 10000 };
				PointRayCaster rayCasterPoint15{ influenceAreaPoint15 }; rayCasterPoint15.generateRays(rng, 1000, true);
				Bvh bvhPoint15{ bvhProperties, influenceAreaPoint15, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPoint15) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Random1000_Point_15_Facing", random1000Triangles, vector{&rayCasterPoint15}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Random1000_Point_15_Facing", scene.buildAndTraverse());
			}

			// === Random1000 scene: 1 point influence area 45 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPoint45{ Pov{{-3,5,-3}, {1,0,1}, 60, 90}, 20, 1, 10000 };
				PointRayCaster rayCasterPoint45{ influenceAreaPoint45 }; rayCasterPoint45.generateRays(rng, 1000, true);
				Bvh bvhPoint45{ bvhProperties, influenceAreaPoint45, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPoint45) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Random1000_Point_45_Facing", random1000Triangles, vector{&rayCasterPoint45}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Random1000_Point_45_Facing", scene.buildAndTraverse());
			}

			// === Random1000 scene: 1 point influence area oblique covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPointOblique{ Pov{{10,12,-3}, {-.55,-.9,1}, 60, 60}, 20, 1, 10000 };
				PointRayCaster rayCasterPointOblique{ influenceAreaPointOblique }; rayCasterPointOblique.generateRays(rng, 1000, true);
				Bvh bvhPointOblique{ bvhProperties, influenceAreaPointOblique, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPointOblique) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Random1000_Point_Oblique_Facing", random1000Triangles, vector{&rayCasterPointOblique}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Random1000_Point_Oblique_Facing", scene.buildAndTraverse());
			}
		}

		// === Sponza scene: 1 point influence area ===
		if constexpr (POINT_FACING and_or SPONZA_SCENE) {
			// === Sponza scene: 1 point influence area parallel to x covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPointParallel{ Pov{{-9,11,-1}, {1,0,0}, 70, 50}, 50, 1, 10000 };
				PointRayCaster rayCasterPointParallel{ influenceAreaPointParallel }; rayCasterPointParallel.generateRays(rng, 1000, true);
				Bvh bvhPointParallel{ bvhProperties, influenceAreaPointParallel, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPointParallel) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Sponza_Point_Parallel_Facing", sponzaTriangles, vector{&rayCasterPointParallel}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Sponza_Point_Parallel_Facing", scene.buildAndTraverse());
			}

			// === Sponza scene: 1 point influence area 15 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPoint15{ Pov{{-12,11,0.5}, {.966,0,-.259}, 70,50}, 60,1, 10000 };
				PointRayCaster rayCasterPoint15{ influenceAreaPoint15 }; rayCasterPoint15.generateRays(rng, 1000, true);
				Bvh bvhPoint15{ bvhProperties, influenceAreaPoint15, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPoint15) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Sponza_Point_15_Facing", sponzaTriangles, vector{&rayCasterPoint15}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Sponza_Point_15_Facing", scene.buildAndTraverse());
			}

			// === Sponza scene: 1 point influence area 45 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPoint45{ Pov{{-10,11,18}, {1,0,-1}, 90,50}, 70,1, 10000 };
				PointRayCaster rayCasterPoint45{ influenceAreaPoint45 }; rayCasterPoint45.generateRays(rng, 1000, true);
				Bvh bvhPoint45{ bvhProperties, influenceAreaPoint45, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPoint45) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Sponza_Point_45_Facing", sponzaTriangles, vector{&rayCasterPoint45}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Sponza_Point_45_Facing", scene.buildAndTraverse());
			}

			// === Sponza scene: 1 point influence area oblique covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPointOblique{ Pov{{-12,10,7.2}, {.84,0.26,-.48}, 70,50}, 70,1, 10000 };
				PointRayCaster rayCasterPointOblique{ influenceAreaPointOblique }; rayCasterPointOblique.generateRays(rng, 1000, true);
				Bvh bvhPointOblique{ bvhProperties, influenceAreaPointOblique, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPointOblique) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Sponza_Point_Oblique_Facing", sponzaTriangles, vector{&rayCasterPointOblique}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Sponza_Point_Oblique_Facing", scene.buildAndTraverse());
			}
		}
	};

	// PLANE SAH+LSPH
	bvhProperties.splitPlaneQualityThreshold = lsphSplitPlaneQualityThreshold;
	{
		// === Wood scene: 1 plane influence area ===
		if constexpr (PLANE_STANDARD and_or WOOD_SCENE) {
			// === Wood scene: 1 plane influence area parallel to x covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlaneParallel{ Plane{{2.5,.9,0}, {-1,0,0}, 1.5,0.9}, 10, 10000 };
				PlaneRayCaster rayCasterPlaneParallel{ influenceAreaPlaneParallel }; rayCasterPlaneParallel.generateRays(rng, 1000, true);
				Bvh bvhPlaneParallel{ bvhProperties, influenceAreaPlaneParallel, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlaneParallel) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Wood_Plane_Parallel_Standard", woodTriangles, vector{&rayCasterPlaneParallel}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Wood_Plane_Parallel_Standard", scene.buildAndTraverse());
			}

			// === Wood scene: 1 plane influence area 15 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlane15{ Plane{{2.5,.8,-.3}, {-.966,0,.259}, 1.6,1}, 10, 10000 };
				PlaneRayCaster rayCasterPlane15{ influenceAreaPlane15 }; rayCasterPlane15.generateRays(rng, 1000, true);
				Bvh bvhPlane15{ bvhProperties, influenceAreaPlane15, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlane15) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Wood_Plane_15_Standard", woodTriangles, vector{&rayCasterPlane15}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Wood_Plane_15_Standard", scene.buildAndTraverse());
			}

			// === Wood scene: 1 plane influence area 45 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlane45{ Plane{{2.5,.8,-3}, {-1,0,1}, 1.5,1}, 10, 10000 };
				PlaneRayCaster rayCasterPlane45{ influenceAreaPlane45 }; rayCasterPlane45.generateRays(rng, 1000, true);
				Bvh bvhPlane45{ bvhProperties, influenceAreaPlane45, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlane45) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Wood_Plane_45_Standard", woodTriangles, vector{&rayCasterPlane45}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Wood_Plane_45_Standard", scene.buildAndTraverse());
			}

			// === Wood scene: 1 plane influence area oblique covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlaneOblique{ Plane{{2.5,1.5,-1}, {-1,-0.3,0.4}, 1.8,1.3}, 20, 10000 };
				PlaneRayCaster rayCasterPlaneOblique{ influenceAreaPlaneOblique }; rayCasterPlaneOblique.generateRays(rng, 1000, true);
				Bvh bvhPlaneOblique{ bvhProperties, influenceAreaPlaneOblique, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlaneOblique) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Wood_Plane_Oblique_Standard", woodTriangles, vector{&rayCasterPlaneOblique}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Wood_Plane_Oblique_Standard", scene.buildAndTraverse());
			}
		}

		// === Suzanne scene: 1 plane influence area ===
		if constexpr (PLANE_STANDARD and_or SUZANNE_SCENE) {
			// === Suzanne scene: 1 plane influence area parallel to x covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlaneParallel{ Plane{{0,0,2}, {0,0,-1}, .9,.9}, 10, 10000 };
				PlaneRayCaster rayCasterPlaneParallel{ influenceAreaPlaneParallel }; rayCasterPlaneParallel.generateRays(rng, 1000, true);
				Bvh bvhPlaneParallel{ bvhProperties, influenceAreaPlaneParallel, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlaneParallel) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "SuzannePlaneParallel_Standard", suzanneTriangles, vector{&rayCasterPlaneParallel}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("SuzannePlaneParallel_Standard", scene.buildAndTraverse());
			}

			// === Suzanne scene: 1 plane influence area 15 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlane15{ Plane{{.5,0,2}, {-.259,0,-.966}, 1,.9}, 10, 10000 };
				PlaneRayCaster rayCasterPlane15{ influenceAreaPlane15 }; rayCasterPlane15.generateRays(rng, 1000, true);
				Bvh bvhPlane15{ bvhProperties, influenceAreaPlane15, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlane15) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "SuzannePlane15_Standard", suzanneTriangles, vector{&rayCasterPlane15}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("SuzannePlane15_Standard", scene.buildAndTraverse());
			}

			// === Suzanne scene: 1 plane influence area 45 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlane45{ Plane{{2,0,2}, {-1,0,-1}, 1.2,.9}, 10, 10000 };
				PlaneRayCaster rayCasterPlane45{ influenceAreaPlane45 }; rayCasterPlane45.generateRays(rng, 1000, true);
				Bvh bvhPlane45{ bvhProperties, influenceAreaPlane45, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlane45) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "SuzannePlane45_Standard", suzanneTriangles, vector{&rayCasterPlane45}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("SuzannePlane45_Standard", scene.buildAndTraverse());
			}

			// === Suzanne scene: 1 plane influence area oblique covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlaneOblique{ Plane{{1,-.5,2}, {-.45,.3,-1}, 2,1.5}, 10, 10000 };
				PlaneRayCaster rayCasterPlaneOblique{ influenceAreaPlaneOblique }; rayCasterPlaneOblique.generateRays(rng, 1000, true);
				Bvh bvhPlaneOblique{ bvhProperties, influenceAreaPlaneOblique, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlaneOblique) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "SuzannePlaneOblique_Standard", suzanneTriangles, vector{&rayCasterPlaneOblique}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("SuzannePlaneOblique_Standard", scene.buildAndTraverse());
			}
		}

		// === Cottage scene: 1 plane influence area ===
		if constexpr (PLANE_STANDARD and_or COTTAGE_SCENE) {
			// === Cottage scene: 1 plane influence area parallel to x covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlaneParallel{ Plane{{10,2.8,0}, {-1,0,0}, 6,2.6}, 35, 10000 };
				PlaneRayCaster rayCasterPlaneParallel{ influenceAreaPlaneParallel }; rayCasterPlaneParallel.generateRays(rng, 1000, true);
				Bvh bvhPlaneParallel{ bvhProperties, influenceAreaPlaneParallel, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlaneParallel) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Cottage_Plane_Parallel_Standard", cottageTriangles, vector{&rayCasterPlaneParallel}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Cottage_Plane_Parallel_Standard", scene.buildAndTraverse());
			}

			// === Cottage scene: 1 plane influence area 15 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlane15{ Plane{{10,2.9,-3}, {-.966,0,.259}, 6,2.9}, 35, 10000 };
				PlaneRayCaster rayCasterPlane15{ influenceAreaPlane15 }; rayCasterPlane15.generateRays(rng, 1000, true);
				Bvh bvhPlane15{ bvhProperties, influenceAreaPlane15, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlane15) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Cottage_Plane_15_Standard", cottageTriangles, vector{&rayCasterPlane15}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Cottage_Plane_15_Standard", scene.buildAndTraverse());
			}

			// === Cottage scene: 1 plane influence area 45 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlane45{ Plane{{7.5,2.9,-7.5}, {-1,0,1}, 8,2.9}, 35, 10000 };
				PlaneRayCaster rayCasterPlane45{ influenceAreaPlane45 }; rayCasterPlane45.generateRays(rng, 1000, true);
				Bvh bvhPlane45{ bvhProperties, influenceAreaPlane45, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlane45) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Cottage_Plane_45_Standard", cottageTriangles, vector{&rayCasterPlane45}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Cottage_Plane_45_Standard", scene.buildAndTraverse());
			}

			// === Cottage scene: 1 plane influence area oblique covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlaneOblique{ Plane{{10,5,-8}, {-1,-.3,.9}, 7,3.3}, 35, 10000 };
				PlaneRayCaster rayCasterPlaneOblique{ influenceAreaPlaneOblique }; rayCasterPlaneOblique.generateRays(rng, 1000, true);
				Bvh bvhPlaneOblique{ bvhProperties, influenceAreaPlaneOblique, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlaneOblique) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Cottage_Plane_Oblique_Standard", cottageTriangles, vector{&rayCasterPlaneOblique}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Cottage_Plane_Oblique_Standard", scene.buildAndTraverse());
			}
		}

		// === CottageWalls scene: 1 plane influence area ===
		if constexpr (PLANE_STANDARD and_or COTTAGE_WALLS_SCENE) {
			// === CottageWalls scene: 1 plane influence area parallel to x covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlaneParallel{ Plane{{10,2.8,0}, {-1,0,0}, 6,2.6}, 35, 10000 };
				PlaneRayCaster rayCasterPlaneParallel{ influenceAreaPlaneParallel }; rayCasterPlaneParallel.generateRays(rng, 1000, true);
				Bvh bvhPlaneParallel{ bvhProperties, influenceAreaPlaneParallel, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlaneParallel) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "CottageWalls_Plane_Parallel_Standard", cottageWallsTriangles, vector{&rayCasterPlaneParallel}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("CottageWalls_Plane_Parallel_Standard", scene.buildAndTraverse());
			}

			// === CottageWalls scene: 1 plane influence area 15 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlane15{ Plane{{10,2.9,-3}, {-.966,0,.259}, 6,2.9}, 35, 10000 };
				PlaneRayCaster rayCasterPlane15{ influenceAreaPlane15 }; rayCasterPlane15.generateRays(rng, 1000, true);
				Bvh bvhPlane15{ bvhProperties, influenceAreaPlane15, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlane15) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "CottageWalls_Plane_15_Standard", cottageWallsTriangles, vector{&rayCasterPlane15}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("CottageWalls_Plane_15_Standard", scene.buildAndTraverse());
			}

			// === CottageWalls scene: 1 plane influence area 45 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlane45{ Plane{{7.5,2.9,-7.5}, {-1,0,1}, 8,2.9}, 35, 10000 };
				PlaneRayCaster rayCasterPlane45{ influenceAreaPlane45 }; rayCasterPlane45.generateRays(rng, 1000, true);
				Bvh bvhPlane45{ bvhProperties, influenceAreaPlane45, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlane45) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "CottageWalls_Plane_45_Standard", cottageWallsTriangles, vector{&rayCasterPlane45}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("CottageWalls_Plane_45_Standard", scene.buildAndTraverse());
			}

			// === CottageWalls scene: 1 plane influence area oblique covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlaneOblique{ Plane{{10,5,-8}, {-1,-.3,.9}, 7,3.3}, 35, 10000 };
				PlaneRayCaster rayCasterPlaneOblique{ influenceAreaPlaneOblique }; rayCasterPlaneOblique.generateRays(rng, 1000, true);
				Bvh bvhPlaneOblique{ bvhProperties, influenceAreaPlaneOblique, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlaneOblique) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "CottageWalls_Plane_Oblique_Standard", cottageWallsTriangles, vector{&rayCasterPlaneOblique}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("CottageWalls_Plane_Oblique_Standard", scene.buildAndTraverse());
			}
		}

		// === Crowd scene: 1 plane influence area ===
		if constexpr (PLANE_STANDARD and_or CROWD_SCENE) {
			// === Crowd scene: 1 plane influence area parallel to x covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlaneParallel{ Plane{{2,2.2,-8}, {0,0,1}, 7,1.8}, 30, 10000 };
				PlaneRayCaster rayCasterPlaneParallel{ influenceAreaPlaneParallel }; rayCasterPlaneParallel.generateRays(rng, 1000, true);
				Bvh bvhPlaneParallel{ bvhProperties, influenceAreaPlaneParallel, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlaneParallel) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Crowd_Plane_Parallel_Standard", crowdTriangles, vector{&rayCasterPlaneParallel}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Crowd_Plane_Parallel_Standard", scene.buildAndTraverse());
			}

			// === Crowd scene: 1 plane influence area 15 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlane15{ Plane{{-2.5,2.2,-8}, {.259,0,.966}, 7.5,1.8}, 30, 10000 };
				PlaneRayCaster rayCasterPlane15{ influenceAreaPlane15 }; rayCasterPlane15.generateRays(rng, 1000, true);
				Bvh bvhPlane15{ bvhProperties, influenceAreaPlane15, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlane15) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Crowd_Plane_15_Standard", crowdTriangles, vector{&rayCasterPlane15}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Crowd_Plane_15_Standard", scene.buildAndTraverse());
			}

			// === Crowd scene: 1 plane influence area 45 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlane45{ Plane{{-8,2.2,-8}, {1,0,1}, 9,1.8}, 30, 10000 };
				PlaneRayCaster rayCasterPlane45{ influenceAreaPlane45 }; rayCasterPlane45.generateRays(rng, 1000, true);
				Bvh bvhPlane45{ bvhProperties, influenceAreaPlane45, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlane45) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Crowd_Plane_45_Standard", crowdTriangles, vector{&rayCasterPlane45}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Crowd_Plane_45_Standard", scene.buildAndTraverse());
			}

			// === Crowd scene: 1 plane influence area oblique covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlaneOblique{ Plane{{-6,7,-8}, {.6,-.4,1}, 7.5,3.4}, 35, 10000 };
				PlaneRayCaster rayCasterPlaneOblique{ influenceAreaPlaneOblique }; rayCasterPlaneOblique.generateRays(rng, 1000, true);
				Bvh bvhPlaneOblique{ bvhProperties, influenceAreaPlaneOblique, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlaneOblique) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Crowd_Plane_Oblique_Standard", crowdTriangles, vector{&rayCasterPlaneOblique}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Crowd_Plane_Oblique_Standard", scene.buildAndTraverse());
			}
		}

		// === Random100 scene: 1 plane influence area ===
		if constexpr (PLANE_STANDARD and_or RANDOM100_SCENE) {
			// === Random100 scene: 1 plane influence area parallel to x covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlaneParallel{ Plane{{5,5,-4}, {0,0,1}, 5,5}, 20, 10000 };
				PlaneRayCaster rayCasterPlaneParallel{ influenceAreaPlaneParallel }; rayCasterPlaneParallel.generateRays(rng, 1000, true);
				Bvh bvhPlaneParallel{ bvhProperties, influenceAreaPlaneParallel, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlaneParallel) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Random100_Plane_Parallel_Standard", random100Triangles, vector{&rayCasterPlaneParallel}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Random100_Plane_Parallel_Standard", scene.buildAndTraverse());
			}

			// === Random100 scene: 1 plane influence area 15 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlane15{ Plane{{3.5,5,-4}, {.259,0,.966}, 6.5,5}, 20, 10000 };
				PlaneRayCaster rayCasterPlane15{ influenceAreaPlane15 }; rayCasterPlane15.generateRays(rng, 1000, true);
				Bvh bvhPlane15{ bvhProperties, influenceAreaPlane15, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlane15) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Random100_Plane_15_Standard", random100Triangles, vector{&rayCasterPlane15}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Random100_Plane_15_Standard", scene.buildAndTraverse());
			}

			// === Random100 scene: 1 plane influence area 45 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlane45{ Plane{{-3,5,-3}, {1,0,1}, 7,5}, 20, 10000 };
				PlaneRayCaster rayCasterPlane45{ influenceAreaPlane45 }; rayCasterPlane45.generateRays(rng, 1000, true);
				Bvh bvhPlane45{ bvhProperties, influenceAreaPlane45, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlane45) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Random100_Plane_45_Standard", random100Triangles, vector{&rayCasterPlane45}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Random100_Plane_45_Standard", scene.buildAndTraverse());
			}

			// === Random100 scene: 1 plane influence area oblique covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlaneOblique{ Plane{{10,12,-3}, {-.55,-.9,1}, 6.5,6.5}, 20, 10000 };
				PlaneRayCaster rayCasterPlaneOblique{ influenceAreaPlaneOblique }; rayCasterPlaneOblique.generateRays(rng, 1000, true);
				Bvh bvhPlaneOblique{ bvhProperties, influenceAreaPlaneOblique, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlaneOblique) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Random100_Plane_Oblique_Standard", random100Triangles, vector{&rayCasterPlaneOblique}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Random100_Plane_Oblique_Standard", scene.buildAndTraverse());
			}
		}

		// === Random1000 scene: 1 plane influence area ===
		if constexpr (PLANE_STANDARD and_or RANDOM1000_SCENE) {
			// === Random1000 scene: 1 plane influence area parallel to x covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlaneParallel{ Plane{{5,5,-4}, {0,0,1}, 5,5}, 20, 10000 };
				PlaneRayCaster rayCasterPlaneParallel{ influenceAreaPlaneParallel }; rayCasterPlaneParallel.generateRays(rng, 1000, true);
				Bvh bvhPlaneParallel{ bvhProperties, influenceAreaPlaneParallel, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlaneParallel) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Random1000_Plane_Parallel_Standard", random1000Triangles, vector{&rayCasterPlaneParallel}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Random1000_Plane_Parallel_Standard", scene.buildAndTraverse());
			}

			// === Random1000 scene: 1 plane influence area 15 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlane15{ Plane{{3.5,5,-4}, {.259,0,.966}, 6.5,5}, 20, 10000 };
				PlaneRayCaster rayCasterPlane15{ influenceAreaPlane15 }; rayCasterPlane15.generateRays(rng, 1000, true);
				Bvh bvhPlane15{ bvhProperties, influenceAreaPlane15, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlane15) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Random1000_Plane_15_Standard", random1000Triangles, vector{&rayCasterPlane15}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Random1000_Plane_15_Standard", scene.buildAndTraverse());
			}

			// === Random1000 scene: 1 plane influence area 45 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlane45{ Plane{{-3,5,-3}, {1,0,1}, 7,5}, 20, 10000 };
				PlaneRayCaster rayCasterPlane45{ influenceAreaPlane45 }; rayCasterPlane45.generateRays(rng, 1000, true);
				Bvh bvhPlane45{ bvhProperties, influenceAreaPlane45, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlane45) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Random1000_Plane_45_Standard", random1000Triangles, vector{&rayCasterPlane45}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Random1000_Plane_45_Standard", scene.buildAndTraverse());
			}

			// === Random1000 scene: 1 plane influence area oblique covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlaneOblique{ Plane{{10,12,-3}, {-.55,-.9,1}, 6.5,6.5}, 20, 10000 };
				PlaneRayCaster rayCasterPlaneOblique{ influenceAreaPlaneOblique }; rayCasterPlaneOblique.generateRays(rng, 1000, true);
				Bvh bvhPlaneOblique{ bvhProperties, influenceAreaPlaneOblique, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlaneOblique) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Random1000_Plane_Oblique_Standard", random1000Triangles, vector{&rayCasterPlaneOblique}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Random1000_Plane_Oblique_Standard", scene.buildAndTraverse());
			}
		}

		// === Sponza scene: 1 plane influence area ===
		if constexpr (PLANE_STANDARD and_or SPONZA_SCENE) {
			// === Sponza scene: 1 plane influence area parallel to x covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlaneParallel{ Plane{{-9,11,-1}, {1,0,0}, 16, 11}, 50, 10000 };
				PlaneRayCaster rayCasterPlaneParallel{ influenceAreaPlaneParallel }; rayCasterPlaneParallel.generateRays(rng, 1000, true);
				Bvh bvhPlaneParallel{ bvhProperties, influenceAreaPlaneParallel, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlaneParallel) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Sponza_Plane_Parallel_Standard", sponzaTriangles, vector{&rayCasterPlaneParallel}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Sponza_Plane_Parallel_Standard", scene.buildAndTraverse());
			}

			// === Sponza scene: 1 plane influence area 15 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlane15{ Plane{{-12,11,0.5}, {.966,0,-.259}, 16,11}, 60, 10000 };
				PlaneRayCaster rayCasterPlane15{ influenceAreaPlane15 }; rayCasterPlane15.generateRays(rng, 1000, true);
				Bvh bvhPlane15{ bvhProperties, influenceAreaPlane15, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlane15) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Sponza_Plane_15_Standard", sponzaTriangles, vector{&rayCasterPlane15}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Sponza_Plane_15_Standard", scene.buildAndTraverse());
			}

			// === Sponza scene: 1 plane influence area 45 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlane45{ Plane{{-10,11,18}, {1,0,-1}, 25,11}, 70, 10000 };
				PlaneRayCaster rayCasterPlane45{ influenceAreaPlane45 }; rayCasterPlane45.generateRays(rng, 1000, true);
				Bvh bvhPlane45{ bvhProperties, influenceAreaPlane45, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlane45) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Sponza_Plane_45_Standard", sponzaTriangles, vector{&rayCasterPlane45}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Sponza_Plane_45_Standard", scene.buildAndTraverse());
			}

			// === Sponza scene: 1 plane influence area oblique covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PlaneInfluenceArea influenceAreaPlaneOblique{ Plane{{-12,10,7.2}, {.84,0.26,-.48}, 16,11}, 70, 10000 };
				PlaneRayCaster rayCasterPlaneOblique{ influenceAreaPlaneOblique }; rayCasterPlaneOblique.generateRays(rng, 1000, true);
				Bvh bvhPlaneOblique{ bvhProperties, influenceAreaPlaneOblique, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPlaneOblique) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Sponza_Plane_Oblique_Standard", sponzaTriangles, vector{&rayCasterPlaneOblique}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Sponza_Plane_Oblique_Standard", scene.buildAndTraverse());
			}
		}
	};

	// POINT SAH+LSPH
	bvhProperties.splitPlaneQualityThreshold = lsphSplitPlaneQualityThreshold;
	{
		// === Wood scene: 1 point influence area ===
		if constexpr (POINT_STANDARD and_or WOOD_SCENE) {
			// === Wood scene: 1 point influence area parallel to x covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPointParallel{ Pov{{2.5,.9,0}, {-1,0,0}, 90, 60}, 10, 1, 10000 };
				PointRayCaster rayCasterPointParallel{ influenceAreaPointParallel }; rayCasterPointParallel.generateRays(rng, 1000, true);
				Bvh bvhPointParallel{ bvhProperties, influenceAreaPointParallel, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPointParallel) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Wood_Point_Parallel_Standard", woodTriangles, vector{&rayCasterPointParallel}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Wood_Point_Parallel_Standard", scene.buildAndTraverse());
			}

			// === Wood scene: 1 point influence area 15 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPoint15{ Pov{{2.5,.8,-.3}, {-.966,0,.259}, 90, 60}, 10, 1, 10000 };
				PointRayCaster rayCasterPoint15{ influenceAreaPoint15 }; rayCasterPoint15.generateRays(rng, 1000, true);
				Bvh bvhPoint15{ bvhProperties, influenceAreaPoint15, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPoint15) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Wood_Point_15_Standard", woodTriangles, vector{&rayCasterPoint15}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Wood_Point_15_Standard", scene.buildAndTraverse());
			}

			// === Wood scene: 1 point influence area 45 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPoint45{ Pov{{2.5,.8,-3}, {-1,0,1}, 90, 60}, 10, 1, 10000 };
				PointRayCaster rayCasterPoint45{ influenceAreaPoint45 }; rayCasterPoint45.generateRays(rng, 1000, true);
				Bvh bvhPoint45{ bvhProperties, influenceAreaPoint45, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPoint45) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Wood_Point_45_Standard", woodTriangles, vector{&rayCasterPoint45}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Wood_Point_45_Standard", scene.buildAndTraverse());
			}

			// === Wood scene: 1 point influence area oblique covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPointOblique{ Pov{{2.5,1.5,-1}, {-1,-0.3,0.4}, 90, 60}, 20, 1, 10000 };
				PointRayCaster rayCasterPointOblique{ influenceAreaPointOblique }; rayCasterPointOblique.generateRays(rng, 1000, true);
				Bvh bvhPointOblique{ bvhProperties, influenceAreaPointOblique, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPointOblique) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Wood_Point_Oblique_Standard", woodTriangles, vector{&rayCasterPointOblique}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Wood_Point_Oblique_Standard", scene.buildAndTraverse());
			}
		}

		// === Suzanne scene: 1 point influence area ===
		if constexpr (POINT_STANDARD and_or SUZANNE_SCENE) {
			// === Suzanne scene: 1 point influence area parallel to x covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPointParallel{ Pov{{0,0,4}, {0,0,-1}, 30, 30}, 10, 2, 10000 };
				PointRayCaster rayCasterPointParallel{ influenceAreaPointParallel }; rayCasterPointParallel.generateRays(rng, 1000, true);
				Bvh bvhPointParallel{ bvhProperties, influenceAreaPointParallel, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPointParallel) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "SuzannePointParallel_Standard", suzanneTriangles, vector{&rayCasterPointParallel}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("SuzannePointParallel_Standard", scene.buildAndTraverse());
			}

			// === Suzanne scene: 1 point influence area 15 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPoint15{ Pov{{1.3,0,5}, {-.259,0,-.966}, 25, 25}, 10, 2, 10000 };
				PointRayCaster rayCasterPoint15{ influenceAreaPoint15 }; rayCasterPoint15.generateRays(rng, 1000, true);
				Bvh bvhPoint15{ bvhProperties, influenceAreaPoint15, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPoint15) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "SuzannePoint15_Standard", suzanneTriangles, vector{&rayCasterPoint15}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("SuzannePoint15_Standard", scene.buildAndTraverse());
			}

			// === Suzanne scene: 1 point influence area 45 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPoint45{ Pov{{3.3,0,3}, {-1,0,-1}, 30, 30}, 10, 2, 10000 };
				PointRayCaster rayCasterPoint45{ influenceAreaPoint45 }; rayCasterPoint45.generateRays(rng, 1000, true);
				Bvh bvhPoint45{ bvhProperties, influenceAreaPoint45, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPoint45) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "SuzannePoint45_Standard", suzanneTriangles, vector{&rayCasterPoint45}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("SuzannePoint45_Standard", scene.buildAndTraverse());
			}

			// === Suzanne scene: 1 point influence area oblique covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPointOblique{ Pov{{2,-1.2,4}, {-.45,.3,-1}, 30, 30}, 10, 2, 10000 };
				PointRayCaster rayCasterPointOblique{ influenceAreaPointOblique }; rayCasterPointOblique.generateRays(rng, 1000, true);
				Bvh bvhPointOblique{ bvhProperties, influenceAreaPointOblique, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPointOblique) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "SuzannePointOblique_Standard", suzanneTriangles, vector{&rayCasterPointOblique}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("SuzannePointOblique_Standard", scene.buildAndTraverse());
			}
		}

		// === Cottage scene: 1 point influence area ===
		if constexpr (POINT_STANDARD and_or COTTAGE_SCENE) {
			// === Cottage scene: 1 point influence area parallel to x covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPointParallel{ Pov{{15,2.7,0}, {-1,0,0}, 70, 35}, 35, 1, 10000 };
				PointRayCaster rayCasterPointParallel{ influenceAreaPointParallel }; rayCasterPointParallel.generateRays(rng, 1000, true);
				Bvh bvhPointParallel{ bvhProperties, influenceAreaPointParallel, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPointParallel) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Cottage_Point_Parallel_Standard", cottageTriangles, vector{&rayCasterPointParallel}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Cottage_Point_Parallel_Standard", scene.buildAndTraverse());
			}

			// === Cottage scene: 1 point influence area 15 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPoint15{ Pov{{15,2.7,-3}, {-.966,0,.259}, 70, 35}, 35, 1, 10000 };
				PointRayCaster rayCasterPoint15{ influenceAreaPoint15 }; rayCasterPoint15.generateRays(rng, 1000, true);
				Bvh bvhPoint15{ bvhProperties, influenceAreaPoint15, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPoint15) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Cottage_Point_15_Standard", cottageTriangles, vector{&rayCasterPoint15}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Cottage_Point_15_Standard", scene.buildAndTraverse());
			}

			// === Cottage scene: 1 point influence area 45 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPoint45{ Pov{{13,2.5,-13}, {-1,0,1}, 50, 20}, 35, 1, 10000 };
				PointRayCaster rayCasterPoint45{ influenceAreaPoint45 }; rayCasterPoint45.generateRays(rng, 1000, true);
				Bvh bvhPoint45{ bvhProperties, influenceAreaPoint45, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPoint45) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Cottage_Point_45_Standard", cottageTriangles, vector{&rayCasterPoint45}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Cottage_Point_45_Standard", scene.buildAndTraverse());
			}

			// === Cottage scene: 1 point influence area oblique covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPointOblique{ Pov{{15,7,-11}, {-1,-.45,.75}, 50, 30}, 35, 1, 10000 };
				PointRayCaster rayCasterPointOblique{ influenceAreaPointOblique }; rayCasterPointOblique.generateRays(rng, 1000, true);
				Bvh bvhPointOblique{ bvhProperties, influenceAreaPointOblique, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPointOblique) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Cottage_Point_Oblique_Standard", cottageTriangles, vector{&rayCasterPointOblique}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Cottage_Point_Oblique_Standard", scene.buildAndTraverse());
			}
		}

		// === CottageWalls scene: 1 point influence area ===
		if constexpr (POINT_STANDARD and_or COTTAGE_WALLS_SCENE) {
			// === CottageWalls scene: 1 point influence area parallel to x covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPointParallel{ Pov{{15,2.7,0}, {-1,0,0}, 70, 35}, 35, 1, 10000 };
				PointRayCaster rayCasterPointParallel{ influenceAreaPointParallel }; rayCasterPointParallel.generateRays(rng, 1000, true);
				Bvh bvhPointParallel{ bvhProperties, influenceAreaPointParallel, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPointParallel) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "CottageWalls_Point_Parallel_Standard", cottageWallsTriangles, vector{&rayCasterPointParallel}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("CottageWalls_Point_Parallel_Standard", scene.buildAndTraverse());
			}

			// === CottageWalls scene: 1 point influence area 15 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPoint15{ Pov{{15,2.7,-3}, {-.966,0,.259}, 70, 35}, 35, 1, 10000 };
				PointRayCaster rayCasterPoint15{ influenceAreaPoint15 }; rayCasterPoint15.generateRays(rng, 1000, true);
				Bvh bvhPoint15{ bvhProperties, influenceAreaPoint15, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPoint15) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "CottageWalls_Point_15_Standard", cottageWallsTriangles, vector{&rayCasterPoint15}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("CottageWalls_Point_15_Standard", scene.buildAndTraverse());
			}

			// === CottageWalls scene: 1 point influence area 45 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPoint45{ Pov{{13,2.5,-13}, {-1,0,1}, 50, 20}, 35, 1, 10000 };
				PointRayCaster rayCasterPoint45{ influenceAreaPoint45 }; rayCasterPoint45.generateRays(rng, 1000, true);
				Bvh bvhPoint45{ bvhProperties, influenceAreaPoint45, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPoint45) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "CottageWalls_Point_45_Standard", cottageWallsTriangles, vector{&rayCasterPoint45}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("CottageWalls_Point_45_Standard", scene.buildAndTraverse());
			}

			// === CottageWalls scene: 1 point influence area oblique covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPointOblique{ Pov{{15,7,-11}, {-1,-.45,.75}, 50, 30}, 35, 1, 10000 };
				PointRayCaster rayCasterPointOblique{ influenceAreaPointOblique }; rayCasterPointOblique.generateRays(rng, 1000, true);
				Bvh bvhPointOblique{ bvhProperties, influenceAreaPointOblique, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPointOblique) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "CottageWalls_Point_Oblique_Standard", cottageWallsTriangles, vector{&rayCasterPointOblique}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("CottageWalls_Point_Oblique_Standard", scene.buildAndTraverse());
			}
		}

		// === Crowd scene: 1 point influence area ===
		if constexpr (POINT_STANDARD and_or CROWD_SCENE) {
			// === Crowd scene: 1 point influence area parallel to x covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPointParallel{ Pov{{2,2.2,-8}, {0,0,1}, 130, 45}, 30, 1, 10000 };
				PointRayCaster rayCasterPointParallel{ influenceAreaPointParallel }; rayCasterPointParallel.generateRays(rng, 1000, true);
				Bvh bvhPointParallel{ bvhProperties, influenceAreaPointParallel, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPointParallel) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Crowd_Point_Parallel_Standard", crowdTriangles, vector{&rayCasterPointParallel}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Crowd_Point_Parallel_Standard", scene.buildAndTraverse());
			}

			// === Crowd scene: 1 point influence area 15 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPoint15{ Pov{{-2.5,2.2,-8}, {.259,0,.966}, 130, 45}, 30, 1, 10000 };
				PointRayCaster rayCasterPoint15{ influenceAreaPoint15 }; rayCasterPoint15.generateRays(rng, 1000, true);
				Bvh bvhPoint15{ bvhProperties, influenceAreaPoint15, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPoint15) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Crowd_Point_15_Standard", crowdTriangles, vector{&rayCasterPoint15}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Crowd_Point_15_Standard", scene.buildAndTraverse());
			}

			// === Crowd scene: 1 point influence area 45 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPoint45{ Pov{{-8,2.2,-8}, {1,0,1}, 130, 45}, 30, 1, 10000 };
				PointRayCaster rayCasterPoint45{ influenceAreaPoint45 }; rayCasterPoint45.generateRays(rng, 1000, true);
				Bvh bvhPoint45{ bvhProperties, influenceAreaPoint45, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPoint45) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Crowd_Point_45_Standard", crowdTriangles, vector{&rayCasterPoint45}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Crowd_Point_45_Standard", scene.buildAndTraverse());
			}

			// === Crowd scene: 1 point influence area oblique covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPointOblique{ Pov{{-6,7,-8}, {.6,-.4,1}, 130, 45}, 35, 1, 10000 };
				PointRayCaster rayCasterPointOblique{ influenceAreaPointOblique }; rayCasterPointOblique.generateRays(rng, 1000, true);
				Bvh bvhPointOblique{ bvhProperties, influenceAreaPointOblique, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPointOblique) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Crowd_Point_Oblique_Standard", crowdTriangles, vector{&rayCasterPointOblique}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Crowd_Point_Oblique_Standard", scene.buildAndTraverse());
			}
		}

		// === Random100 scene: 1 point influence area ===
		if constexpr (POINT_STANDARD and_or RANDOM100_SCENE) {
			// === Random100 scene: 1 point influence area parallel to x covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPointParallel{ Pov{{5,5,-4}, {0,0,1}, 90, 90}, 20, 1, 10000 };
				PointRayCaster rayCasterPointParallel{ influenceAreaPointParallel }; rayCasterPointParallel.generateRays(rng, 1000, true);
				Bvh bvhPointParallel{ bvhProperties, influenceAreaPointParallel, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPointParallel) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Random100_Point_Parallel_Standard", random100Triangles, vector{&rayCasterPointParallel}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Random100_Point_Parallel_Standard", scene.buildAndTraverse());
			}

			// === Random100 scene: 1 point influence area 15 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPoint15{ Pov{{3.5,5,-4}, {.259,0,.966}, 90, 90}, 20, 1, 10000 };
				PointRayCaster rayCasterPoint15{ influenceAreaPoint15 }; rayCasterPoint15.generateRays(rng, 1000, true);
				Bvh bvhPoint15{ bvhProperties, influenceAreaPoint15, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPoint15) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Random100_Point_15_Standard", random100Triangles, vector{&rayCasterPoint15}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Random100_Point_15_Standard", scene.buildAndTraverse());
			}

			// === Random100 scene: 1 point influence area 45 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPoint45{ Pov{{-3,5,-3}, {1,0,1}, 60, 90}, 20, 1, 10000 };
				PointRayCaster rayCasterPoint45{ influenceAreaPoint45 }; rayCasterPoint45.generateRays(rng, 1000, true);
				Bvh bvhPoint45{ bvhProperties, influenceAreaPoint45, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPoint45) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Random100_Point_45_Standard", random100Triangles, vector{&rayCasterPoint45}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Random100_Point_45_Standard", scene.buildAndTraverse());
			}

			// === Random100 scene: 1 point influence area oblique covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPointOblique{ Pov{{10,12,-3}, {-.55,-.9,1}, 60, 60}, 20, 1, 10000 };
				PointRayCaster rayCasterPointOblique{ influenceAreaPointOblique }; rayCasterPointOblique.generateRays(rng, 1000, true);
				Bvh bvhPointOblique{ bvhProperties, influenceAreaPointOblique, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPointOblique) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Random100_Point_Oblique_Standard", random100Triangles, vector{&rayCasterPointOblique}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Random100_Point_Oblique_Standard", scene.buildAndTraverse());
			}
		}

		// === Random1000 scene: 1 point influence area ===
		if constexpr (POINT_STANDARD and_or RANDOM1000_SCENE) {
			// === Random1000 scene: 1 point influence area parallel to x covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPointParallel{ Pov{{5,5,-4}, {0,0,1}, 90, 90}, 20, 1, 10000 };
				PointRayCaster rayCasterPointParallel{ influenceAreaPointParallel }; rayCasterPointParallel.generateRays(rng, 1000, true);
				Bvh bvhPointParallel{ bvhProperties, influenceAreaPointParallel, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPointParallel) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Random1000_Point_Parallel_Standard", random1000Triangles, vector{&rayCasterPointParallel}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Random1000_Point_Parallel_Standard", scene.buildAndTraverse());
			}

			// === Random1000 scene: 1 point influence area 15 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPoint15{ Pov{{3.5,5,-4}, {.259,0,.966}, 90, 90}, 20, 1, 10000 };
				PointRayCaster rayCasterPoint15{ influenceAreaPoint15 }; rayCasterPoint15.generateRays(rng, 1000, true);
				Bvh bvhPoint15{ bvhProperties, influenceAreaPoint15, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPoint15) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Random1000_Point_15_Standard", random1000Triangles, vector{&rayCasterPoint15}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Random1000_Point_15_Standard", scene.buildAndTraverse());
			}

			// === Random1000 scene: 1 point influence area 45 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPoint45{ Pov{{-3,5,-3}, {1,0,1}, 60, 90}, 20, 1, 10000 };
				PointRayCaster rayCasterPoint45{ influenceAreaPoint45 }; rayCasterPoint45.generateRays(rng, 1000, true);
				Bvh bvhPoint45{ bvhProperties, influenceAreaPoint45, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPoint45) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Random1000_Point_45_Standard", random1000Triangles, vector{&rayCasterPoint45}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Random1000_Point_45_Standard", scene.buildAndTraverse());
			}

			// === Random1000 scene: 1 point influence area oblique covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPointOblique{ Pov{{10,12,-3}, {-.55,-.9,1}, 60, 60}, 20, 1, 10000 };
				PointRayCaster rayCasterPointOblique{ influenceAreaPointOblique }; rayCasterPointOblique.generateRays(rng, 1000, true);
				Bvh bvhPointOblique{ bvhProperties, influenceAreaPointOblique, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPointOblique) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Random1000_Point_Oblique_Standard", random1000Triangles, vector{&rayCasterPointOblique}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Random1000_Point_Oblique_Standard", scene.buildAndTraverse());
			}
		}

		// === Sponza scene: 1 point influence area ===
		if constexpr (POINT_STANDARD and_or SPONZA_SCENE) {
			// === Sponza scene: 1 point influence area parallel to x covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPointParallel{ Pov{{-9,11,-1}, {1,0,0}, 70, 50}, 50, 1, 10000 };
				PointRayCaster rayCasterPointParallel{ influenceAreaPointParallel }; rayCasterPointParallel.generateRays(rng, 1000, true);
				Bvh bvhPointParallel{ bvhProperties, influenceAreaPointParallel, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPointParallel) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Sponza_Point_Parallel_Standard", sponzaTriangles, vector{&rayCasterPointParallel}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Sponza_Point_Parallel_Standard", scene.buildAndTraverse());
			}

			// === Sponza scene: 1 point influence area 15 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPoint15{ Pov{{-12,11,0.5}, {.966,0,-.259}, 70,50}, 60,1, 10000 };
				PointRayCaster rayCasterPoint15{ influenceAreaPoint15 }; rayCasterPoint15.generateRays(rng, 1000, true);
				Bvh bvhPoint15{ bvhProperties, influenceAreaPoint15, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPoint15) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Sponza_Point_15_Standard", sponzaTriangles, vector{&rayCasterPoint15}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Sponza_Point_15_Standard", scene.buildAndTraverse());
			}

			// === Sponza scene: 1 point influence area 45 degrees covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPoint45{ Pov{{-10,11,18}, {1,0,-1}, 90,50}, 70,1, 10000 };
				PointRayCaster rayCasterPoint45{ influenceAreaPoint45 }; rayCasterPoint45.generateRays(rng, 1000, true);
				Bvh bvhPoint45{ bvhProperties, influenceAreaPoint45, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPoint45) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Sponza_Point_45_Standard", sponzaTriangles, vector{&rayCasterPoint45}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Sponza_Point_45_Standard", scene.buildAndTraverse());
			}

			// === Sponza scene: 1 point influence area oblique covering all the scene. 1 ray caster relative to the influence area. ===
			{
				PointInfluenceArea influenceAreaPointOblique{ Pov{{-12,10,7.2}, {.84,0.26,-.48}, 70,50}, 70,1, 10000 };
				PointRayCaster rayCasterPointOblique{ influenceAreaPointOblique }; rayCasterPointOblique.generateRays(rng, 1000, true);
				Bvh bvhPointOblique{ bvhProperties, influenceAreaPointOblique, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "point" };
				TopLevelOctree topLevel{ octreeProperties, fallbackBvh, std::move(bvhPointOblique) };
				auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Sponza_Point_Oblique_Standard", sponzaTriangles, vector{&rayCasterPointOblique}, std::move(topLevel), topLevelAnalyzer };
				csvTraversal.addAnalysis("Sponza_Point_Oblique_Standard", scene.buildAndTraverse());
			}
		}
	};
	
	csvTraversal.generateCsv("D:/Users/lapof/Documents/Development/ProjectedAreaHeuristic/Results/ExportedCsvPlaneSponza4.csv");
#endif //BVH_TESTS

#define OCTREE_TESTS 0
#if OCTREE_TESTS
	// OCTREE VS AABBS
	{
		distributions::UniformBoxDistribution mainDistribution3d{ 2,4, 2,6, 2,3 };
		distributions::UniformBoxDistribution otherDistribution3d{ -1,1, -1,1 , -1,1 };
		const vector triangles = Triangle::generateRandom(50, rng, mainDistribution3d, otherDistribution3d);
		constexpr int MAX_ITERATIONS = 1000;
		constexpr int MAX_INFLUENCE_AREAS = 10;
		int octreeHits = 0; //how many influence areas are hit in the octree
		int aabbsHits = 0; //how many influence areas are hit in the aabbs

		//create a vector of influence areas and the associated BVHs
		list<PlaneInfluenceArea> planeInfluenceAreas{}; //do NOT use vector. We must store a reference to the elements of these lists inside Bvh, and vector can reallocate memory
		list<PointInfluenceArea> pointInfluenceAreas{}; //do NOT use vector. We must store a reference to the elements of these lists inside Bvh, and vector can reallocate memory
		vector<Bvh> bvhsOctree{};
		vector<Bvh> bvhsAabbs{};
		distributions::UniformBoxDistribution position{ 2,6, 2,6, 2,6 };
		distributions::UniformBoxDistribution direction{ -1,1, -1,1, -1,1 };
		distributions::UniformBoxDistribution size{ 1,5, 1,5, 2,10 };
		distributions::UniformBoxDistribution fovAndNear{ 20,100, 20,100, 1,2 };
		for (int i = 0; i < MAX_INFLUENCE_AREAS; ++i) {
			Vector3 sizePlane = size(rng);
			Vector3 sizePoint = size(rng);
			Vector3 fov = fovAndNear(rng);

			planeInfluenceAreas.emplace_back(Plane{ position(rng), direction(rng), sizePlane.x, sizePlane.y }, sizePlane.z, 10000);
			pointInfluenceAreas.emplace_back(Pov{ position(rng), direction(rng), fov.x, fov.y }, fov.z + sizePoint.z, fov.z, 10000);

			bvhsOctree.emplace_back(Bvh{bvhProperties, planeInfluenceAreas.back(), bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "plane"});
			bvhsOctree.emplace_back(Bvh{bvhProperties, pointInfluenceAreas.back(), bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "point"});
			
			bvhsAabbs.emplace_back(Bvh{bvhProperties, planeInfluenceAreas.back(), bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "plane"});
			bvhsAabbs.emplace_back(Bvh{bvhProperties, pointInfluenceAreas.back(), bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "point"});
		}

		octreeProperties.maxLevel = 4;
		TopLevelOctree topLevelOctree{ octreeProperties, fallbackBvh };
		for (auto& elem : bvhsOctree) {
			topLevelOctree.addBvh(std::move(elem));
		}
		topLevelOctree.build(triangles);
		utilities::TimeLogger octreeTime{ [](DurationMs duration) { cout << endl << "Top level octree duration in ms: " << duration.count(); } };
		for (int i = 0; i < MAX_ITERATIONS; ++i) {
			Vector3 point = mainDistribution3d(rng);
			auto res = topLevelOctree.containedIn(point);
			octreeHits += res.size();
		}
		octreeTime.stop();

		TopLevelAabbs topLevelAabbs{ fallbackBvh };
		for (auto& elem : bvhsAabbs) {
			topLevelAabbs.addBvh(std::move(elem));
		}
		topLevelAabbs.build(triangles);
		utilities::TimeLogger aabbsTime{ [](DurationMs duration) { cout << endl << "Top level AABBs duration in ms: " << duration.count(); } };
		for (int i = 0; i < MAX_ITERATIONS; ++i) {
			Vector3 point = mainDistribution3d(rng);
			auto res = topLevelAabbs.containedIn(point);
			aabbsHits += res.size();
		}
		aabbsTime.stop();

		cout << endl << "Influence areas hit --> octree: " << octreeHits << "\taabbs: " << aabbsHits;
	}
#endif //OCTREE_TESTS

	return 0;
}
