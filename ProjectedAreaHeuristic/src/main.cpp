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
	CsvExporter csvExporter{ 
		ACCESSOR("Estimated PAH cost",					AT::TOP_LEVEL, ["bvhs"].at(0)["globalInfo"]["pahCost"]),
		ACCESSOR("Estimated SAH cost",					AT::TOP_LEVEL, ["bvhs"].back()["globalInfo"]["sahCost"]),
		ACCESSOR("PAH cost with fallback",				AT::PAH, ["cost"]["traversalCostAveragePerRay"]),
		ACCESSOR("PAH intersections with fallback",		AT::PAH, ["total"]["intersectionTests"]["intersectionTestsAveragePerRay"]),
		ACCESSOR("PAH hit percentage",					AT::PAH, ["total"]["hitMiss"]["hitsPercentage"]),
		ACCESSOR("PAH cost without fallback",			AT::PAH, ["cost"]["traversalCostForBvhPerRay"].at(0).at(1)),
		ACCESSOR("PAH intersections without fallback",	AT::PAH, ["fallback"]["intersectionTests"]["intersectionTestsNonFallbackAveragePerRay"]),
		ACCESSOR("SAH cost without fallback",			AT::FALLBACK, ["cost"]["traversalCostAveragePerRay"]),
		ACCESSOR("SAH intersections without fallback",	AT::FALLBACK, ["fallback"]["intersectionTests"]["intersectionTestsNonFallbackAveragePerRay"]),
		ACCESSOR("Max level PAH",						AT::TOP_LEVEL, ["bvhs"].at(0)["globalInfo"]["maxLevel"]),
		ACCESSOR("Max leaf cost",						AT::TOP_LEVEL, ["bvhs"].at(0)["globalInfo"]["properties"]["maxLeafCost"]),
		ACCESSOR("Max leaf area",						AT::TOP_LEVEL, ["bvhs"].at(0)["globalInfo"]["properties"]["maxLeafArea"]),
		ACCESSOR("Max leaf hit probability",			AT::TOP_LEVEL, ["bvhs"].at(0)["globalInfo"]["properties"]["maxLeafHitProbability"]),
		ACCESSOR("Max triangles per leaf",				AT::TOP_LEVEL, ["bvhs"].at(0)["globalInfo"]["properties"]["maxTrianglesPerLeaf"]),
		ACCESSOR("Max non fallback levels",				AT::TOP_LEVEL, ["bvhs"].at(0)["globalInfo"]["properties"]["maxNonFallbackLevels"]),
		ACCESSOR("Split plane quality threshold",		AT::TOP_LEVEL, ["bvhs"].at(0)["globalInfo"]["properties"]["splitPlaneQualityThreshold"]),
		ACCESSOR("Max children/father hit probability",	AT::TOP_LEVEL, ["bvhs"].at(0)["globalInfo"]["properties"]["maxChildrenFatherHitProbabilityRatio"])
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
	.maxLeafCost = 0.0f,
	.maxLeafArea = 0.0f,
	.maxLeafHitProbability = 0.0f,
	.maxTrianglesPerLeaf = 2,
	.maxLevels = 100,
	.bins = 40,
	.maxNonFallbackLevels = 100,
	.splitPlaneQualityThreshold = 0.0f,
	.maxChildrenFatherHitProbabilityRatio = 1.3f
	};

	//top level structure properties used in most scenes
	TopLevel::Properties topLevelProperties{
	.splitPlaneQualityThreshold = 0.0f,
	.maxChildrenFatherHitProbabilityRatio = 1.3f
	};

	//fallback BVH used by most top level structures
	Bvh fallbackBvh{ bvhProperties, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesLongest<0.5f>, bvhStrategies::shouldStopThresholdOrLevel, "fallback" };


	// Filter test scenes to run
	constexpr bool ALL = true;
	constexpr bool PLANE_FULL_PARALLEL = ALL || false;
	constexpr bool PLANE_FULL_PARALLEL_LONGEST = ALL || false;
	constexpr bool WOOD_SCENE = ALL || false;
	constexpr bool SUZANNE_SCENE = ALL || true;
	constexpr bool COTTAGE_SCENE = ALL || false;
	constexpr bool COTTAGE_WALLS_SCENE = ALL || false;
	constexpr bool RANDOM100_SCENE = ALL || false;
	constexpr bool RANDOM1000_SCENE = ALL || false;
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
			csvExporter.addAnalysis("WoodPlaneFullParallel", scene.buildAndTraverse());
		}

		// === Wood scene: 1 plane influence area 15 degrees covering all the scene. 1 ray caster relative to the influence area. ===
		{
			PlaneInfluenceArea influenceAreaPlaneFull15{ Plane{{2.5,.8,-.3}, {-.966,0,.259}}, {1.6,1}, 10, 10000 };
			PlaneRayCaster rayCasterPlaneFull15{ influenceAreaPlaneFull15 }; rayCasterPlaneFull15.generateRays(rng, 1000, true);
			Bvh bvhPlaneFull15{ bvhProperties, influenceAreaPlaneFull15, bvhStrategies::computeCostPah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
			TopLevelOctree topLevel{ topLevelProperties, TopLevelOctree::OctreeProperties{ 4, false }, fallbackBvh, std::move(bvhPlaneFull15) };
			auto scene = TestScene{ string(RESULTS_DIRECTORY) + "WoodPlaneFull15", woodTriangles, vector{&rayCasterPlaneFull15}, std::move(topLevel), topLevelAnalyzer };
			csvExporter.addAnalysis("WoodPlaneFull15", scene.buildAndTraverse());
		}

		// === Wood scene: 1 plane influence area 45 degrees covering all the scene. 1 ray caster relative to the influence area. ===
		{
			PlaneInfluenceArea influenceAreaPlaneFull45{ Plane{{2.5,.8,-3}, {-1,0,1}}, {1.5,1}, 10, 10000 };
			PlaneRayCaster rayCasterPlaneFull45{ influenceAreaPlaneFull45 }; rayCasterPlaneFull45.generateRays(rng, 1000, true);
			Bvh bvhPlaneFull45{ bvhProperties, influenceAreaPlaneFull45, bvhStrategies::computeCostPah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
			TopLevelOctree topLevel{ topLevelProperties, TopLevelOctree::OctreeProperties{ 4, false }, fallbackBvh, std::move(bvhPlaneFull45) };
			auto scene = TestScene{ string(RESULTS_DIRECTORY) + "WoodPlaneFull45", woodTriangles, vector{&rayCasterPlaneFull45}, std::move(topLevel), topLevelAnalyzer };
			csvExporter.addAnalysis("WoodPlaneFull45", scene.buildAndTraverse());
		}

		// === Wood scene: 1 plane influence area oblique covering all the scene. 1 ray caster relative to the influence area. ===
		{
			PlaneInfluenceArea influenceAreaPlaneFullOblique{ Plane{{2.5,1.5,-1}, {-1,-0.3,0.4}}, {1.8,1.3}, 20, 10000 };
			PlaneRayCaster rayCasterPlaneFullOblique{ influenceAreaPlaneFullOblique }; rayCasterPlaneFullOblique.generateRays(rng, 1000, true);
			Bvh bvhPlaneFullOblique{ bvhProperties, influenceAreaPlaneFullOblique, bvhStrategies::computeCostPah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
			TopLevelOctree topLevel{ topLevelProperties, TopLevelOctree::OctreeProperties{ 4, false }, fallbackBvh, std::move(bvhPlaneFullOblique) };
			auto scene = TestScene{ string(RESULTS_DIRECTORY) + "WoodPlaneFullOblique", woodTriangles, vector{&rayCasterPlaneFullOblique}, std::move(topLevel), topLevelAnalyzer };
			csvExporter.addAnalysis("WoodPlaneFullOblique", scene.buildAndTraverse());
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
			csvExporter.addAnalysis("SuzannePlaneFullParallel", scene.buildAndTraverse());
		}

		// === Suzanne scene: 1 plane influence area 15 degrees covering all the scene. 1 ray caster relative to the influence area. ===
		{
			PlaneInfluenceArea influenceAreaPlaneFull15{ Plane{{.5,0,2}, {-.259,0,-.966}}, {1,.9}, 10, 10000 };
			PlaneRayCaster rayCasterPlaneFull15{ influenceAreaPlaneFull15 }; rayCasterPlaneFull15.generateRays(rng, 1000, true);
			Bvh bvhPlaneFull15{ bvhProperties, influenceAreaPlaneFull15, bvhStrategies::computeCostPah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
			TopLevelOctree topLevel{ topLevelProperties, TopLevelOctree::OctreeProperties{ 4, false }, fallbackBvh, std::move(bvhPlaneFull15) };
			auto scene = TestScene{ string(RESULTS_DIRECTORY) + "SuzannePlaneFull15", suzanneTriangles, vector{&rayCasterPlaneFull15}, std::move(topLevel), topLevelAnalyzer };
			csvExporter.addAnalysis("SuzannePlaneFull15", scene.buildAndTraverse());
		}

		// === Suzanne scene: 1 plane influence area 45 degrees covering all the scene. 1 ray caster relative to the influence area. ===
		{
			PlaneInfluenceArea influenceAreaPlaneFull45{ Plane{{2,0,2}, {-1,0,-1}}, {1.2,.9}, 10, 10000 };
			PlaneRayCaster rayCasterPlaneFull45{ influenceAreaPlaneFull45 }; rayCasterPlaneFull45.generateRays(rng, 1000, true);
			Bvh bvhPlaneFull45{ bvhProperties, influenceAreaPlaneFull45, bvhStrategies::computeCostPah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
			TopLevelOctree topLevel{ topLevelProperties, TopLevelOctree::OctreeProperties{ 4, false }, fallbackBvh, std::move(bvhPlaneFull45) };
			auto scene = TestScene{ string(RESULTS_DIRECTORY) + "SuzannePlaneFull45", suzanneTriangles, vector{&rayCasterPlaneFull45}, std::move(topLevel), topLevelAnalyzer };
			csvExporter.addAnalysis("SuzannePlaneFull45", scene.buildAndTraverse());
		}

		// === Suzanne scene: 1 plane influence area oblique covering all the scene. 1 ray caster relative to the influence area. ===
		{
			PlaneInfluenceArea influenceAreaPlaneFullOblique{ Plane{{1,-.5,2}, {-.45,.3,-1}}, {.9,.9}, 10, 10000 };
			PlaneRayCaster rayCasterPlaneFullOblique{ influenceAreaPlaneFullOblique }; rayCasterPlaneFullOblique.generateRays(rng, 1000, true);
			Bvh bvhPlaneFullOblique{ bvhProperties, influenceAreaPlaneFullOblique, bvhStrategies::computeCostPah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
			TopLevelOctree topLevel{ topLevelProperties, TopLevelOctree::OctreeProperties{ 4, false }, fallbackBvh, std::move(bvhPlaneFullOblique) };
			auto scene = TestScene{ string(RESULTS_DIRECTORY) + "SuzannePlaneFullOblique", suzanneTriangles, vector{&rayCasterPlaneFullOblique}, std::move(topLevel), topLevelAnalyzer };
			csvExporter.addAnalysis("SuzannePlaneFullOblique", scene.buildAndTraverse());
		}
	}

	// === Cottage scene: 1 plane full influence area ===
	if constexpr (PLANE_FULL_PARALLEL || COTTAGE_SCENE) {
		// === Cottage scene: 1 plane influence area parallel to x covering all the scene. 1 ray caster relative to the influence area. ===
		{
			PlaneInfluenceArea influenceAreaPlaneFullParallel{ Plane{{10,2.8,0}, {-1,0,0}}, {6,2.6}, 35, 10000 };
			PlaneRayCaster rayCasterPlaneFullParallel{ influenceAreaPlaneFullParallel }; rayCasterPlaneFullParallel.generateRays(rng, 1000, true);
			Bvh bvhPlaneFullParallel{ bvhProperties, influenceAreaPlaneFullParallel, bvhStrategies::computeCostPah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
			TopLevelOctree topLevel{ topLevelProperties, TopLevelOctree::OctreeProperties{ 4, false }, fallbackBvh, std::move(bvhPlaneFullParallel) };
			auto scene = TestScene{ string(RESULTS_DIRECTORY) + "CottagePlaneFullParallel", cottageTriangles, vector{&rayCasterPlaneFullParallel}, std::move(topLevel), topLevelAnalyzer };
			csvExporter.addAnalysis("CottagePlaneFullParallel", scene.buildAndTraverse());
		}

		// === Cottage scene: 1 plane influence area 15 degrees covering all the scene. 1 ray caster relative to the influence area. ===
		{
			PlaneInfluenceArea influenceAreaPlaneFull15{ Plane{{10,2.9,-3}, {-.966,0,.259}}, {6,2.9}, 35, 10000 };
			PlaneRayCaster rayCasterPlaneFull15{ influenceAreaPlaneFull15 }; rayCasterPlaneFull15.generateRays(rng, 1000, true);
			Bvh bvhPlaneFull15{ bvhProperties, influenceAreaPlaneFull15, bvhStrategies::computeCostPah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
			TopLevelOctree topLevel{ topLevelProperties, TopLevelOctree::OctreeProperties{ 4, false }, fallbackBvh, std::move(bvhPlaneFull15) };
			auto scene = TestScene{ string(RESULTS_DIRECTORY) + "CottagePlaneFull15", cottageTriangles, vector{&rayCasterPlaneFull15}, std::move(topLevel), topLevelAnalyzer };
			csvExporter.addAnalysis("CottagePlaneFull15", scene.buildAndTraverse());
		}

		// === Cottage scene: 1 plane influence area 45 degrees covering all the scene. 1 ray caster relative to the influence area. ===
		{
			PlaneInfluenceArea influenceAreaPlaneFull45{ Plane{{7.5,2.9,-7.5}, {-1,0,1}}, {8,2.9}, 35, 10000 };
			PlaneRayCaster rayCasterPlaneFull45{ influenceAreaPlaneFull45 }; rayCasterPlaneFull45.generateRays(rng, 1000, true);
			Bvh bvhPlaneFull45{ bvhProperties, influenceAreaPlaneFull45, bvhStrategies::computeCostPah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
			TopLevelOctree topLevel{ topLevelProperties, TopLevelOctree::OctreeProperties{ 4, false }, fallbackBvh, std::move(bvhPlaneFull45) };
			auto scene = TestScene{ string(RESULTS_DIRECTORY) + "CottagePlaneFull45", cottageTriangles, vector{&rayCasterPlaneFull45}, std::move(topLevel), topLevelAnalyzer };
			csvExporter.addAnalysis("CottagePlaneFull45", scene.buildAndTraverse());
		}

		// === Cottage scene: 1 plane influence area oblique covering all the scene. 1 ray caster relative to the influence area. ===
		{
			PlaneInfluenceArea influenceAreaPlaneFullOblique{ Plane{{10,5,-8}, {-1,-.3,.9}}, {7,3.3}, 35, 10000 };
			PlaneRayCaster rayCasterPlaneFullOblique{ influenceAreaPlaneFullOblique }; rayCasterPlaneFullOblique.generateRays(rng, 1000, true);
			Bvh bvhPlaneFullOblique{ bvhProperties, influenceAreaPlaneFullOblique, bvhStrategies::computeCostPah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
			TopLevelOctree topLevel{ topLevelProperties, TopLevelOctree::OctreeProperties{ 4, false }, fallbackBvh, std::move(bvhPlaneFullOblique) };
			auto scene = TestScene{ string(RESULTS_DIRECTORY) + "CottagePlaneFullOblique", cottageTriangles, vector{&rayCasterPlaneFullOblique}, std::move(topLevel), topLevelAnalyzer };
			csvExporter.addAnalysis("CottagePlaneFullOblique", scene.buildAndTraverse());
		}
	}
	
	// === CottageWalls scene: 1 plane full influence area ===
	if constexpr (PLANE_FULL_PARALLEL || COTTAGE_WALLS_SCENE) {
		// === CottageWalls scene: 1 plane influence area parallel to x covering all the scene. 1 ray caster relative to the influence area. ===
		{
			PlaneInfluenceArea influenceAreaPlaneFullParallel{ Plane{{10,2.8,0}, {-1,0,0}}, {6,2.6}, 35, 10000 };
			PlaneRayCaster rayCasterPlaneFullParallel{ influenceAreaPlaneFullParallel }; rayCasterPlaneFullParallel.generateRays(rng, 1000, true);
			Bvh bvhPlaneFullParallel{ bvhProperties, influenceAreaPlaneFullParallel, bvhStrategies::computeCostPah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
			TopLevelOctree topLevel{ topLevelProperties, TopLevelOctree::OctreeProperties{ 4, false }, fallbackBvh, std::move(bvhPlaneFullParallel) };
			auto scene = TestScene{ string(RESULTS_DIRECTORY) + "CottageWallsPlaneFullParallel", cottageWallsTriangles, vector{&rayCasterPlaneFullParallel}, std::move(topLevel), topLevelAnalyzer };
			csvExporter.addAnalysis("CottageWallsPlaneFullParallel", scene.buildAndTraverse());
		}

		// === CottageWalls scene: 1 plane influence area 15 degrees covering all the scene. 1 ray caster relative to the influence area. ===
		{
			PlaneInfluenceArea influenceAreaPlaneFull15{ Plane{{10,2.9,-3}, {-.966,0,.259}}, {6,2.9}, 35, 10000 };
			PlaneRayCaster rayCasterPlaneFull15{ influenceAreaPlaneFull15 }; rayCasterPlaneFull15.generateRays(rng, 1000, true);
			Bvh bvhPlaneFull15{ bvhProperties, influenceAreaPlaneFull15, bvhStrategies::computeCostPah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
			TopLevelOctree topLevel{ topLevelProperties, TopLevelOctree::OctreeProperties{ 4, false }, fallbackBvh, std::move(bvhPlaneFull15) };
			auto scene = TestScene{ string(RESULTS_DIRECTORY) + "CottageWallsPlaneFull15", cottageWallsTriangles, vector{&rayCasterPlaneFull15}, std::move(topLevel), topLevelAnalyzer };
			csvExporter.addAnalysis("CottageWallsPlaneFull15", scene.buildAndTraverse());
		}

		// === CottageWalls scene: 1 plane influence area 45 degrees covering all the scene. 1 ray caster relative to the influence area. ===
		{
			PlaneInfluenceArea influenceAreaPlaneFull45{ Plane{{7.5,2.9,-7.5}, {-1,0,1}}, {8,2.9}, 35, 10000 };
			PlaneRayCaster rayCasterPlaneFull45{ influenceAreaPlaneFull45 }; rayCasterPlaneFull45.generateRays(rng, 1000, true);
			Bvh bvhPlaneFull45{ bvhProperties, influenceAreaPlaneFull45, bvhStrategies::computeCostPah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
			TopLevelOctree topLevel{ topLevelProperties, TopLevelOctree::OctreeProperties{ 4, false }, fallbackBvh, std::move(bvhPlaneFull45) };
			auto scene = TestScene{ string(RESULTS_DIRECTORY) + "CottageWallsPlaneFull45", cottageWallsTriangles, vector{&rayCasterPlaneFull45}, std::move(topLevel), topLevelAnalyzer };
			csvExporter.addAnalysis("CottageWallsPlaneFull45", scene.buildAndTraverse());
		}

		// === CottageWalls scene: 1 plane influence area oblique covering all the scene. 1 ray caster relative to the influence area. ===
		{
			PlaneInfluenceArea influenceAreaPlaneFullOblique{ Plane{{10,5,-8}, {-1,-.3,.9}}, {7,3.3}, 35, 10000 };
			PlaneRayCaster rayCasterPlaneFullOblique{ influenceAreaPlaneFullOblique }; rayCasterPlaneFullOblique.generateRays(rng, 1000, true);
			Bvh bvhPlaneFullOblique{ bvhProperties, influenceAreaPlaneFullOblique, bvhStrategies::computeCostPah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
			TopLevelOctree topLevel{ topLevelProperties, TopLevelOctree::OctreeProperties{ 4, false }, fallbackBvh, std::move(bvhPlaneFullOblique) };
			auto scene = TestScene{ string(RESULTS_DIRECTORY) + "CottageWallsPlaneFullOblique", cottageWallsTriangles, vector{&rayCasterPlaneFullOblique}, std::move(topLevel), topLevelAnalyzer };
			csvExporter.addAnalysis("CottageWallsPlaneFullOblique", scene.buildAndTraverse());
		}
	}

	// === Random100 scene: 1 plane full influence area ===
	if constexpr (PLANE_FULL_PARALLEL || RANDOM100_SCENE) {
		// === Random100 scene: 1 plane influence area parallel to x covering all the scene. 1 ray caster relative to the influence area. ===
		{
			PlaneInfluenceArea influenceAreaPlaneFullParallel{ Plane{{5,5,-4}, {0,0,1}}, {5,5}, 20, 10000 };
			PlaneRayCaster rayCasterPlaneFullParallel{ influenceAreaPlaneFullParallel }; rayCasterPlaneFullParallel.generateRays(rng, 1000, true);
			Bvh bvhPlaneFullParallel{ bvhProperties, influenceAreaPlaneFullParallel, bvhStrategies::computeCostPah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
			TopLevelOctree topLevel{ topLevelProperties, TopLevelOctree::OctreeProperties{ 4, false }, fallbackBvh, std::move(bvhPlaneFullParallel) };
			auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Random100PlaneFullParallel", random100Triangles, vector{&rayCasterPlaneFullParallel}, std::move(topLevel), topLevelAnalyzer };
			csvExporter.addAnalysis("Random100PlaneFullParallel", scene.buildAndTraverse());
		}

		// === Random100 scene: 1 plane influence area 15 degrees covering all the scene. 1 ray caster relative to the influence area. ===
		{
			PlaneInfluenceArea influenceAreaPlaneFull15{ Plane{{3.5,5,-4}, {.259,0,.966}}, {6.5,5}, 20, 10000 };
			PlaneRayCaster rayCasterPlaneFull15{ influenceAreaPlaneFull15 }; rayCasterPlaneFull15.generateRays(rng, 1000, true);
			Bvh bvhPlaneFull15{ bvhProperties, influenceAreaPlaneFull15, bvhStrategies::computeCostPah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
			TopLevelOctree topLevel{ topLevelProperties, TopLevelOctree::OctreeProperties{ 4, false }, fallbackBvh, std::move(bvhPlaneFull15) };
			auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Random100PlaneFull15", random100Triangles, vector{&rayCasterPlaneFull15}, std::move(topLevel), topLevelAnalyzer };
			csvExporter.addAnalysis("Random100PlaneFull15", scene.buildAndTraverse());
		}

		// === Random100 scene: 1 plane influence area 45 degrees covering all the scene. 1 ray caster relative to the influence area. ===
		{
			PlaneInfluenceArea influenceAreaPlaneFull45{ Plane{{-3,5,-3}, {1,0,1}}, {7,5}, 20, 10000 };
			PlaneRayCaster rayCasterPlaneFull45{ influenceAreaPlaneFull45 }; rayCasterPlaneFull45.generateRays(rng, 1000, true);
			Bvh bvhPlaneFull45{ bvhProperties, influenceAreaPlaneFull45, bvhStrategies::computeCostPah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
			TopLevelOctree topLevel{ topLevelProperties, TopLevelOctree::OctreeProperties{ 4, false }, fallbackBvh, std::move(bvhPlaneFull45) };
			auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Random100PlaneFull45", random100Triangles, vector{&rayCasterPlaneFull45}, std::move(topLevel), topLevelAnalyzer };
			csvExporter.addAnalysis("Random100PlaneFull45", scene.buildAndTraverse());
		}

		// === Random100 scene: 1 plane influence area oblique covering all the scene. 1 ray caster relative to the influence area. ===
		{
			PlaneInfluenceArea influenceAreaPlaneFullOblique{ Plane{{10,12,-3}, {-.55,-.9,1}}, {6.5,6.5}, 20, 10000 };
			PlaneRayCaster rayCasterPlaneFullOblique{ influenceAreaPlaneFullOblique }; rayCasterPlaneFullOblique.generateRays(rng, 1000, true);
			Bvh bvhPlaneFullOblique{ bvhProperties, influenceAreaPlaneFullOblique, bvhStrategies::computeCostPah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
			TopLevelOctree topLevel{ topLevelProperties, TopLevelOctree::OctreeProperties{ 4, false }, fallbackBvh, std::move(bvhPlaneFullOblique) };
			auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Random100PlaneFullOblique", random100Triangles, vector{&rayCasterPlaneFullOblique}, std::move(topLevel), topLevelAnalyzer };
			csvExporter.addAnalysis("Random100PlaneFullOblique", scene.buildAndTraverse());
		}
	}

	// === Random1000 scene: 1 plane full influence area ===
	if constexpr (PLANE_FULL_PARALLEL || RANDOM1000_SCENE) {
		// === Random1000 scene: 1 plane influence area parallel to x covering all the scene. 1 ray caster relative to the influence area. ===
		{
			PlaneInfluenceArea influenceAreaPlaneFullParallel{ Plane{{5,5,-4}, {0,0,1}}, {5,5}, 20, 10000 };
			PlaneRayCaster rayCasterPlaneFullParallel{ influenceAreaPlaneFullParallel }; rayCasterPlaneFullParallel.generateRays(rng, 1000, true);
			Bvh bvhPlaneFullParallel{ bvhProperties, influenceAreaPlaneFullParallel, bvhStrategies::computeCostPah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
			TopLevelOctree topLevel{ topLevelProperties, TopLevelOctree::OctreeProperties{ 4, false }, fallbackBvh, std::move(bvhPlaneFullParallel) };
			auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Random1000PlaneFullParallel", random1000Triangles, vector{&rayCasterPlaneFullParallel}, std::move(topLevel), topLevelAnalyzer };
			csvExporter.addAnalysis("Random1000PlaneFullParallel", scene.buildAndTraverse());
		}

		// === Random1000 scene: 1 plane influence area 15 degrees covering all the scene. 1 ray caster relative to the influence area. ===
		{
			PlaneInfluenceArea influenceAreaPlaneFull15{ Plane{{3.5,5,-4}, {.259,0,.966}}, {6.5,5}, 20, 10000 };
			PlaneRayCaster rayCasterPlaneFull15{ influenceAreaPlaneFull15 }; rayCasterPlaneFull15.generateRays(rng, 1000, true);
			Bvh bvhPlaneFull15{ bvhProperties, influenceAreaPlaneFull15, bvhStrategies::computeCostPah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
			TopLevelOctree topLevel{ topLevelProperties, TopLevelOctree::OctreeProperties{ 4, false }, fallbackBvh, std::move(bvhPlaneFull15) };
			auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Random1000PlaneFull15", random1000Triangles, vector{&rayCasterPlaneFull15}, std::move(topLevel), topLevelAnalyzer };
			csvExporter.addAnalysis("Random1000PlaneFull15", scene.buildAndTraverse());
		}

		// === Random1000 scene: 1 plane influence area 45 degrees covering all the scene. 1 ray caster relative to the influence area. ===
		{
			PlaneInfluenceArea influenceAreaPlaneFull45{ Plane{{-3,5,-3}, {1,0,1}}, {7,5}, 20, 10000 };
			PlaneRayCaster rayCasterPlaneFull45{ influenceAreaPlaneFull45 }; rayCasterPlaneFull45.generateRays(rng, 1000, true);
			Bvh bvhPlaneFull45{ bvhProperties, influenceAreaPlaneFull45, bvhStrategies::computeCostPah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
			TopLevelOctree topLevel{ topLevelProperties, TopLevelOctree::OctreeProperties{ 4, false }, fallbackBvh, std::move(bvhPlaneFull45) };
			auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Random1000PlaneFull45", random1000Triangles, vector{&rayCasterPlaneFull45}, std::move(topLevel), topLevelAnalyzer };
			csvExporter.addAnalysis("Random1000PlaneFull45", scene.buildAndTraverse());
		}

		// === Random1000 scene: 1 plane influence area oblique covering all the scene. 1 ray caster relative to the influence area. ===
		{
			PlaneInfluenceArea influenceAreaPlaneFullOblique{ Plane{{10,12,-3}, {-.55,-.9,1}}, {6.5,6.5}, 20, 10000 };
			PlaneRayCaster rayCasterPlaneFullOblique{ influenceAreaPlaneFullOblique }; rayCasterPlaneFullOblique.generateRays(rng, 1000, true);
			Bvh bvhPlaneFullOblique{ bvhProperties, influenceAreaPlaneFullOblique, bvhStrategies::computeCostPah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
			TopLevelOctree topLevel{ topLevelProperties, TopLevelOctree::OctreeProperties{ 4, false }, fallbackBvh, std::move(bvhPlaneFullOblique) };
			auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Random1000PlaneFullOblique", random1000Triangles, vector{&rayCasterPlaneFullOblique}, std::move(topLevel), topLevelAnalyzer };
			csvExporter.addAnalysis("Random1000PlaneFullOblique", scene.buildAndTraverse());
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
			csvExporter.addAnalysis("WoodPlaneFullParallelLongest", scene.buildAndTraverse());
		}

		// === Wood scene: 1 plane influence area 15 degrees covering all the scene. 1 ray caster relative to the influence area. ===
		{
			PlaneInfluenceArea influenceAreaPlaneFull15{ Plane{{2.5,.8,-.3}, {-.966,0,.259}}, {1.6,1}, 10, 10000 };
			PlaneRayCaster rayCasterPlaneFull15{ influenceAreaPlaneFull15 }; rayCasterPlaneFull15.generateRays(rng, 1000, true);
			Bvh bvhPlaneFull15{ bvhProperties, influenceAreaPlaneFull15, bvhStrategies::computeCostPah, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
			TopLevelOctree topLevel{ topLevelProperties, TopLevelOctree::OctreeProperties{ 4, false }, fallbackBvh, std::move(bvhPlaneFull15) };
			auto scene = TestScene{ string(RESULTS_DIRECTORY) + "WoodPlaneFull15Longest", woodTriangles, vector{&rayCasterPlaneFull15}, std::move(topLevel), topLevelAnalyzer };
			csvExporter.addAnalysis("WoodPlaneFull15Longest", scene.buildAndTraverse());
		}

		// === Wood scene: 1 plane influence area 45 degrees covering all the scene. 1 ray caster relative to the influence area. ===
		{
			PlaneInfluenceArea influenceAreaPlaneFull45{ Plane{{2.5,.8,-3}, {-1,0,1}}, {1.5,1}, 10, 10000 };
			PlaneRayCaster rayCasterPlaneFull45{ influenceAreaPlaneFull45 }; rayCasterPlaneFull45.generateRays(rng, 1000, true);
			Bvh bvhPlaneFull45{ bvhProperties, influenceAreaPlaneFull45, bvhStrategies::computeCostPah, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
			TopLevelOctree topLevel{ topLevelProperties, TopLevelOctree::OctreeProperties{ 4, false }, fallbackBvh, std::move(bvhPlaneFull45) };
			auto scene = TestScene{ string(RESULTS_DIRECTORY) + "WoodPlaneFull45Longest", woodTriangles, vector{&rayCasterPlaneFull45}, std::move(topLevel), topLevelAnalyzer };
			csvExporter.addAnalysis("WoodPlaneFull45Longest", scene.buildAndTraverse());
		}

		// === Wood scene: 1 plane influence area oblique covering all the scene. 1 ray caster relative to the influence area. ===
		{
			PlaneInfluenceArea influenceAreaPlaneFullOblique{ Plane{{2.5,1.5,-1}, {-1,-0.3,0.4}}, {1.8,1.3}, 20, 10000 };
			PlaneRayCaster rayCasterPlaneFullOblique{ influenceAreaPlaneFullOblique }; rayCasterPlaneFullOblique.generateRays(rng, 1000, true);
			Bvh bvhPlaneFullOblique{ bvhProperties, influenceAreaPlaneFullOblique, bvhStrategies::computeCostPah, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
			TopLevelOctree topLevel{ topLevelProperties, TopLevelOctree::OctreeProperties{ 4, false }, fallbackBvh, std::move(bvhPlaneFullOblique) };
			auto scene = TestScene{ string(RESULTS_DIRECTORY) + "WoodPlaneFullObliqueLongest", woodTriangles, vector{&rayCasterPlaneFullOblique}, std::move(topLevel), topLevelAnalyzer };
			csvExporter.addAnalysis("WoodPlaneFullObliqueLongest", scene.buildAndTraverse());
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
			csvExporter.addAnalysis("SuzannePlaneFullParallelLongest", scene.buildAndTraverse());
		}

		// === Suzanne scene: 1 plane influence area 15 degrees covering all the scene. 1 ray caster relative to the influence area. ===
		{
			PlaneInfluenceArea influenceAreaPlaneFull15{ Plane{{.5,0,2}, {-.259,0,-.966}}, {1,.9}, 10, 10000 };
			PlaneRayCaster rayCasterPlaneFull15{ influenceAreaPlaneFull15 }; rayCasterPlaneFull15.generateRays(rng, 1000, true);
			Bvh bvhPlaneFull15{ bvhProperties, influenceAreaPlaneFull15, bvhStrategies::computeCostPah, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
			TopLevelOctree topLevel{ topLevelProperties, TopLevelOctree::OctreeProperties{ 4, false }, fallbackBvh, std::move(bvhPlaneFull15) };
			auto scene = TestScene{ string(RESULTS_DIRECTORY) + "SuzannePlaneFull15Longest", suzanneTriangles, vector{&rayCasterPlaneFull15}, std::move(topLevel), topLevelAnalyzer };
			csvExporter.addAnalysis("SuzannePlaneFull15Longest", scene.buildAndTraverse());
		}

		// === Suzanne scene: 1 plane influence area 45 degrees covering all the scene. 1 ray caster relative to the influence area. ===
		{
			PlaneInfluenceArea influenceAreaPlaneFull45{ Plane{{2,0,2}, {-1,0,-1}}, {1.2,.9}, 10, 10000 };
			PlaneRayCaster rayCasterPlaneFull45{ influenceAreaPlaneFull45 }; rayCasterPlaneFull45.generateRays(rng, 1000, true);
			Bvh bvhPlaneFull45{ bvhProperties, influenceAreaPlaneFull45, bvhStrategies::computeCostPah, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
			TopLevelOctree topLevel{ topLevelProperties, TopLevelOctree::OctreeProperties{ 4, false }, fallbackBvh, std::move(bvhPlaneFull45) };
			auto scene = TestScene{ string(RESULTS_DIRECTORY) + "SuzannePlaneFull45Longest", suzanneTriangles, vector{&rayCasterPlaneFull45}, std::move(topLevel), topLevelAnalyzer };
			csvExporter.addAnalysis("SuzannePlaneFull45Longest", scene.buildAndTraverse());
		}

		// === Suzanne scene: 1 plane influence area oblique covering all the scene. 1 ray caster relative to the influence area. ===
		{
			PlaneInfluenceArea influenceAreaPlaneFullOblique{ Plane{{1,-.5,2}, {-.45,.3,-1}}, {.9,.9}, 10, 10000 };
			PlaneRayCaster rayCasterPlaneFullOblique{ influenceAreaPlaneFullOblique }; rayCasterPlaneFullOblique.generateRays(rng, 1000, true);
			Bvh bvhPlaneFullOblique{ bvhProperties, influenceAreaPlaneFullOblique, bvhStrategies::computeCostPah, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
			TopLevelOctree topLevel{ topLevelProperties, TopLevelOctree::OctreeProperties{ 4, false }, fallbackBvh, std::move(bvhPlaneFullOblique) };
			auto scene = TestScene{ string(RESULTS_DIRECTORY) + "SuzannePlaneFullObliqueLongest", suzanneTriangles, vector{&rayCasterPlaneFullOblique}, std::move(topLevel), topLevelAnalyzer };
			csvExporter.addAnalysis("SuzannePlaneFullObliqueLongest", scene.buildAndTraverse());
		}
	}

	// === Cottage scene: 1 plane full influence area ===
	if constexpr (PLANE_FULL_PARALLEL || COTTAGE_SCENE) {
		// === Cottage scene: 1 plane influence area parallel to x covering all the scene. 1 ray caster relative to the influence area. ===
		{
			PlaneInfluenceArea influenceAreaPlaneFullParallel{ Plane{{10,2.8,0}, {-1,0,0}}, {6,2.6}, 35, 10000 };
			PlaneRayCaster rayCasterPlaneFullParallel{ influenceAreaPlaneFullParallel }; rayCasterPlaneFullParallel.generateRays(rng, 1000, true);
			Bvh bvhPlaneFullParallel{ bvhProperties, influenceAreaPlaneFullParallel, bvhStrategies::computeCostPah, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
			TopLevelOctree topLevel{ topLevelProperties, TopLevelOctree::OctreeProperties{ 4, false }, fallbackBvh, std::move(bvhPlaneFullParallel) };
			auto scene = TestScene{ string(RESULTS_DIRECTORY) + "CottagePlaneFullParallelLongest", cottageTriangles, vector{&rayCasterPlaneFullParallel}, std::move(topLevel), topLevelAnalyzer };
			csvExporter.addAnalysis("CottagePlaneFullParallelLongest", scene.buildAndTraverse());
		}

		// === Cottage scene: 1 plane influence area 15 degrees covering all the scene. 1 ray caster relative to the influence area. ===
		{
			PlaneInfluenceArea influenceAreaPlaneFull15{ Plane{{10,2.9,-3}, {-.966,0,.259}}, {6,2.9}, 35, 10000 };
			PlaneRayCaster rayCasterPlaneFull15{ influenceAreaPlaneFull15 }; rayCasterPlaneFull15.generateRays(rng, 1000, true);
			Bvh bvhPlaneFull15{ bvhProperties, influenceAreaPlaneFull15, bvhStrategies::computeCostPah, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
			TopLevelOctree topLevel{ topLevelProperties, TopLevelOctree::OctreeProperties{ 4, false }, fallbackBvh, std::move(bvhPlaneFull15) };
			auto scene = TestScene{ string(RESULTS_DIRECTORY) + "CottagePlaneFull15Longest", cottageTriangles, vector{&rayCasterPlaneFull15}, std::move(topLevel), topLevelAnalyzer };
			csvExporter.addAnalysis("CottagePlaneFull15Longest", scene.buildAndTraverse());
		}

		// === Cottage scene: 1 plane influence area 45 degrees covering all the scene. 1 ray caster relative to the influence area. ===
		{
			PlaneInfluenceArea influenceAreaPlaneFull45{ Plane{{7.5,2.9,-7.5}, {-1,0,1}}, {8,2.9}, 35, 10000 };
			PlaneRayCaster rayCasterPlaneFull45{ influenceAreaPlaneFull45 }; rayCasterPlaneFull45.generateRays(rng, 1000, true);
			Bvh bvhPlaneFull45{ bvhProperties, influenceAreaPlaneFull45, bvhStrategies::computeCostPah, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
			TopLevelOctree topLevel{ topLevelProperties, TopLevelOctree::OctreeProperties{ 4, false }, fallbackBvh, std::move(bvhPlaneFull45) };
			auto scene = TestScene{ string(RESULTS_DIRECTORY) + "CottagePlaneFull45Longest", cottageTriangles, vector{&rayCasterPlaneFull45}, std::move(topLevel), topLevelAnalyzer };
			csvExporter.addAnalysis("CottagePlaneFull45Longest", scene.buildAndTraverse());
		}

		// === Cottage scene: 1 plane influence area oblique covering all the scene. 1 ray caster relative to the influence area. ===
		{
			PlaneInfluenceArea influenceAreaPlaneFullOblique{ Plane{{10,5,-8}, {-1,-.3,.9}}, {7,3.3}, 35, 10000 };
			PlaneRayCaster rayCasterPlaneFullOblique{ influenceAreaPlaneFullOblique }; rayCasterPlaneFullOblique.generateRays(rng, 1000, true);
			Bvh bvhPlaneFullOblique{ bvhProperties, influenceAreaPlaneFullOblique, bvhStrategies::computeCostPah, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
			TopLevelOctree topLevel{ topLevelProperties, TopLevelOctree::OctreeProperties{ 4, false }, fallbackBvh, std::move(bvhPlaneFullOblique) };
			auto scene = TestScene{ string(RESULTS_DIRECTORY) + "CottagePlaneFullObliqueLongest", cottageTriangles, vector{&rayCasterPlaneFullOblique}, std::move(topLevel), topLevelAnalyzer };
			csvExporter.addAnalysis("CottagePlaneFullObliqueLongest", scene.buildAndTraverse());
		}
	}

	// === CottageWalls scene: 1 plane full influence area ===
	if constexpr (PLANE_FULL_PARALLEL || COTTAGE_WALLS_SCENE) {
		// === CottageWalls scene: 1 plane influence area parallel to x covering all the scene. 1 ray caster relative to the influence area. ===
		{
			PlaneInfluenceArea influenceAreaPlaneFullParallel{ Plane{{10,2.8,0}, {-1,0,0}}, {6,2.6}, 35, 10000 };
			PlaneRayCaster rayCasterPlaneFullParallel{ influenceAreaPlaneFullParallel }; rayCasterPlaneFullParallel.generateRays(rng, 1000, true);
			Bvh bvhPlaneFullParallel{ bvhProperties, influenceAreaPlaneFullParallel, bvhStrategies::computeCostPah, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
			TopLevelOctree topLevel{ topLevelProperties, TopLevelOctree::OctreeProperties{ 4, false }, fallbackBvh, std::move(bvhPlaneFullParallel) };
			auto scene = TestScene{ string(RESULTS_DIRECTORY) + "CottageWallsPlaneFullParallelLongest", cottageWallsTriangles, vector{&rayCasterPlaneFullParallel}, std::move(topLevel), topLevelAnalyzer };
			csvExporter.addAnalysis("CottageWallsPlaneFullParallelLongest", scene.buildAndTraverse());
		}

		// === CottageWalls scene: 1 plane influence area 15 degrees covering all the scene. 1 ray caster relative to the influence area. ===
		{
			PlaneInfluenceArea influenceAreaPlaneFull15{ Plane{{10,2.9,-3}, {-.966,0,.259}}, {6,2.9}, 35, 10000 };
			PlaneRayCaster rayCasterPlaneFull15{ influenceAreaPlaneFull15 }; rayCasterPlaneFull15.generateRays(rng, 1000, true);
			Bvh bvhPlaneFull15{ bvhProperties, influenceAreaPlaneFull15, bvhStrategies::computeCostPah, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
			TopLevelOctree topLevel{ topLevelProperties, TopLevelOctree::OctreeProperties{ 4, false }, fallbackBvh, std::move(bvhPlaneFull15) };
			auto scene = TestScene{ string(RESULTS_DIRECTORY) + "CottageWallsPlaneFull15Longest", cottageWallsTriangles, vector{&rayCasterPlaneFull15}, std::move(topLevel), topLevelAnalyzer };
			csvExporter.addAnalysis("CottageWallsPlaneFull15Longest", scene.buildAndTraverse());
		}

		// === CottageWalls scene: 1 plane influence area 45 degrees covering all the scene. 1 ray caster relative to the influence area. ===
		{
			PlaneInfluenceArea influenceAreaPlaneFull45{ Plane{{7.5,2.9,-7.5}, {-1,0,1}}, {8,2.9}, 35, 10000 };
			PlaneRayCaster rayCasterPlaneFull45{ influenceAreaPlaneFull45 }; rayCasterPlaneFull45.generateRays(rng, 1000, true);
			Bvh bvhPlaneFull45{ bvhProperties, influenceAreaPlaneFull45, bvhStrategies::computeCostPah, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
			TopLevelOctree topLevel{ topLevelProperties, TopLevelOctree::OctreeProperties{ 4, false }, fallbackBvh, std::move(bvhPlaneFull45) };
			auto scene = TestScene{ string(RESULTS_DIRECTORY) + "CottageWallsPlaneFull45Longest", cottageWallsTriangles, vector{&rayCasterPlaneFull45}, std::move(topLevel), topLevelAnalyzer };
			csvExporter.addAnalysis("CottageWallsPlaneFull45Longest", scene.buildAndTraverse());
		}

		// === CottageWalls scene: 1 plane influence area oblique covering all the scene. 1 ray caster relative to the influence area. ===
		{
			PlaneInfluenceArea influenceAreaPlaneFullOblique{ Plane{{10,5,-8}, {-1,-.3,.9}}, {7,3.3}, 35, 10000 };
			PlaneRayCaster rayCasterPlaneFullOblique{ influenceAreaPlaneFullOblique }; rayCasterPlaneFullOblique.generateRays(rng, 1000, true);
			Bvh bvhPlaneFullOblique{ bvhProperties, influenceAreaPlaneFullOblique, bvhStrategies::computeCostPah, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
			TopLevelOctree topLevel{ topLevelProperties, TopLevelOctree::OctreeProperties{ 4, false }, fallbackBvh, std::move(bvhPlaneFullOblique) };
			auto scene = TestScene{ string(RESULTS_DIRECTORY) + "CottageWallsPlaneFullObliqueLongest", cottageWallsTriangles, vector{&rayCasterPlaneFullOblique}, std::move(topLevel), topLevelAnalyzer };
			csvExporter.addAnalysis("CottageWallsPlaneFullObliqueLongest", scene.buildAndTraverse());
		}
	}

	// === Random100 scene: 1 plane full influence area longest split ===
	if constexpr (PLANE_FULL_PARALLEL_LONGEST || RANDOM100_SCENE) {
		// === Random100 scene: 1 plane influence area parallel to x covering all the scene. 1 ray caster relative to the influence area. ===
		{
			PlaneInfluenceArea influenceAreaPlaneFullParallel{ Plane{{5,5,-4}, {0,0,1}}, {5,5}, 20, 10000 };
			PlaneRayCaster rayCasterPlaneFullParallel{ influenceAreaPlaneFullParallel }; rayCasterPlaneFullParallel.generateRays(rng, 1000, true);
			Bvh bvhPlaneFullParallel{ bvhProperties, influenceAreaPlaneFullParallel, bvhStrategies::computeCostPah, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
			TopLevelOctree topLevel{ topLevelProperties, TopLevelOctree::OctreeProperties{ 4, false }, fallbackBvh, std::move(bvhPlaneFullParallel) };
			auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Random100PlaneFullParallelLongest", random100Triangles, vector{&rayCasterPlaneFullParallel}, std::move(topLevel), topLevelAnalyzer };
			csvExporter.addAnalysis("Random100PlaneFullParallelLongest", scene.buildAndTraverse());
		}

		// === Random100 scene: 1 plane influence area 15 degrees covering all the scene. 1 ray caster relative to the influence area. ===
		{
			PlaneInfluenceArea influenceAreaPlaneFull15{ Plane{{3.5,5,-4}, {.259,0,.966}}, {6.5,5}, 20, 10000 };
			PlaneRayCaster rayCasterPlaneFull15{ influenceAreaPlaneFull15 }; rayCasterPlaneFull15.generateRays(rng, 1000, true);
			Bvh bvhPlaneFull15{ bvhProperties, influenceAreaPlaneFull15, bvhStrategies::computeCostPah, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
			TopLevelOctree topLevel{ topLevelProperties, TopLevelOctree::OctreeProperties{ 4, false }, fallbackBvh, std::move(bvhPlaneFull15) };
			auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Random100PlaneFull15Longest", random100Triangles, vector{&rayCasterPlaneFull15}, std::move(topLevel), topLevelAnalyzer };
			csvExporter.addAnalysis("Random100PlaneFull15Longest", scene.buildAndTraverse());
		}

		// === Random100 scene: 1 plane influence area 45 degrees covering all the scene. 1 ray caster relative to the influence area. ===
		{
			PlaneInfluenceArea influenceAreaPlaneFull45{ Plane{{-3,5,-3}, {1,0,1}}, {7,5}, 20, 10000 };
			PlaneRayCaster rayCasterPlaneFull45{ influenceAreaPlaneFull45 }; rayCasterPlaneFull45.generateRays(rng, 1000, true);
			Bvh bvhPlaneFull45{ bvhProperties, influenceAreaPlaneFull45, bvhStrategies::computeCostPah, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
			TopLevelOctree topLevel{ topLevelProperties, TopLevelOctree::OctreeProperties{ 4, false }, fallbackBvh, std::move(bvhPlaneFull45) };
			auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Random100PlaneFull45Longest", random100Triangles, vector{&rayCasterPlaneFull45}, std::move(topLevel), topLevelAnalyzer };
			csvExporter.addAnalysis("Random100PlaneFull45Longest", scene.buildAndTraverse());
		}

		// === Random100 scene: 1 plane influence area oblique covering all the scene. 1 ray caster relative to the influence area. ===
		{
			PlaneInfluenceArea influenceAreaPlaneFullOblique{ Plane{{10,12,-3}, {-.55,-.9,1}}, {6.5,6.5}, 20, 10000 };
			PlaneRayCaster rayCasterPlaneFullOblique{ influenceAreaPlaneFullOblique }; rayCasterPlaneFullOblique.generateRays(rng, 1000, true);
			Bvh bvhPlaneFullOblique{ bvhProperties, influenceAreaPlaneFullOblique, bvhStrategies::computeCostPah, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
			TopLevelOctree topLevel{ topLevelProperties, TopLevelOctree::OctreeProperties{ 4, false }, fallbackBvh, std::move(bvhPlaneFullOblique) };
			auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Random100PlaneFullObliqueLongest", random100Triangles, vector{&rayCasterPlaneFullOblique}, std::move(topLevel), topLevelAnalyzer };
			csvExporter.addAnalysis("Random100PlaneFullObliqueLongest", scene.buildAndTraverse());
		}
	}

	// === Random1000 scene: 1 plane full influence area longest split ===
	if constexpr (PLANE_FULL_PARALLEL_LONGEST || RANDOM1000_SCENE) {
		// === Random1000 scene: 1 plane influence area parallel to x covering all the scene. 1 ray caster relative to the influence area. ===
		{
			PlaneInfluenceArea influenceAreaPlaneFullParallel{ Plane{{5,5,-4}, {0,0,1}}, {5,5}, 20, 10000 };
			PlaneRayCaster rayCasterPlaneFullParallel{ influenceAreaPlaneFullParallel }; rayCasterPlaneFullParallel.generateRays(rng, 1000, true);
			Bvh bvhPlaneFullParallel{ bvhProperties, influenceAreaPlaneFullParallel, bvhStrategies::computeCostPah, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
			TopLevelOctree topLevel{ topLevelProperties, TopLevelOctree::OctreeProperties{ 4, false }, fallbackBvh, std::move(bvhPlaneFullParallel) };
			auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Random1000PlaneFullParallelLongest", random1000Triangles, vector{&rayCasterPlaneFullParallel}, std::move(topLevel), topLevelAnalyzer };
			csvExporter.addAnalysis("Random1000PlaneFullParallelLongest", scene.buildAndTraverse());
		}

		// === Random1000 scene: 1 plane influence area 15 degrees covering all the scene. 1 ray caster relative to the influence area. ===
		{
			PlaneInfluenceArea influenceAreaPlaneFull15{ Plane{{3.5,5,-4}, {.259,0,.966}}, {6.5,5}, 20, 10000 };
			PlaneRayCaster rayCasterPlaneFull15{ influenceAreaPlaneFull15 }; rayCasterPlaneFull15.generateRays(rng, 1000, true);
			Bvh bvhPlaneFull15{ bvhProperties, influenceAreaPlaneFull15, bvhStrategies::computeCostPah, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
			TopLevelOctree topLevel{ topLevelProperties, TopLevelOctree::OctreeProperties{ 4, false }, fallbackBvh, std::move(bvhPlaneFull15) };
			auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Random1000PlaneFull15Longest", random1000Triangles, vector{&rayCasterPlaneFull15}, std::move(topLevel), topLevelAnalyzer };
			csvExporter.addAnalysis("Random1000PlaneFull15Longest", scene.buildAndTraverse());
		}

		// === Random1000 scene: 1 plane influence area 45 degrees covering all the scene. 1 ray caster relative to the influence area. ===
		{
			PlaneInfluenceArea influenceAreaPlaneFull45{ Plane{{-3,5,-3}, {1,0,1}}, {7,5}, 20, 10000 };
			PlaneRayCaster rayCasterPlaneFull45{ influenceAreaPlaneFull45 }; rayCasterPlaneFull45.generateRays(rng, 1000, true);
			Bvh bvhPlaneFull45{ bvhProperties, influenceAreaPlaneFull45, bvhStrategies::computeCostPah, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
			TopLevelOctree topLevel{ topLevelProperties, TopLevelOctree::OctreeProperties{ 4, false }, fallbackBvh, std::move(bvhPlaneFull45) };
			auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Random1000PlaneFull45Longest", random1000Triangles, vector{&rayCasterPlaneFull45}, std::move(topLevel), topLevelAnalyzer };
			csvExporter.addAnalysis("Random1000PlaneFull45Longest", scene.buildAndTraverse());
		}

		// === Random1000 scene: 1 plane influence area oblique covering all the scene. 1 ray caster relative to the influence area. ===
		{
			PlaneInfluenceArea influenceAreaPlaneFullOblique{ Plane{{10,12,-3}, {-.55,-.9,1}}, {6.5,6.5}, 20, 10000 };
			PlaneRayCaster rayCasterPlaneFullOblique{ influenceAreaPlaneFullOblique }; rayCasterPlaneFullOblique.generateRays(rng, 1000, true);
			Bvh bvhPlaneFullOblique{ bvhProperties, influenceAreaPlaneFullOblique, bvhStrategies::computeCostPah, bvhStrategies::chooseSplittingPlanesLongest, bvhStrategies::shouldStopThresholdOrLevel, "plane" };
			TopLevelOctree topLevel{ topLevelProperties, TopLevelOctree::OctreeProperties{ 4, false }, fallbackBvh, std::move(bvhPlaneFullOblique) };
			auto scene = TestScene{ string(RESULTS_DIRECTORY) + "Random1000PlaneFullObliqueLongest", random1000Triangles, vector{&rayCasterPlaneFullOblique}, std::move(topLevel), topLevelAnalyzer };
			csvExporter.addAnalysis("Random1000PlaneFullObliqueLongest", scene.buildAndTraverse());
		}
	}


	csvExporter.generateCsv("D:/Users/lapof/Documents/Development/ProjectedAreaHeuristic/Results/ExportedCsv.csv");
	return 0;
}
