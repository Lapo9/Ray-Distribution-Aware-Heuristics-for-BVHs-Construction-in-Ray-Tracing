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
	distributions::UniformBoxDistribution mainDistribution3d{ 0,10, 0,10, 0,10 };
	distributions::UniformBoxDistribution otherDistribution3d{ -1,1, -1,1 , -1,1 };
	//auto triangles = Triangle::generateRandom(1000, rng, mainDistribution3d, otherDistribution3d);
	auto triangles = Triangle::fromObj("D:/Users/lapof/Documents/Development/ProjectedAreaHeuristic/ProjectedAreaHeuristic/exampleData/WoodScene.obj");

	//create influence areas
	PlaneInfluenceArea planeInfluenceArea1{ Plane{{-1.5,0,-1.5}, {1,0,1}}, {.5,.5}, 4, 10000 };
	PlaneInfluenceArea planeInfluenceArea2{ Plane{{2.5,.8,0}, {-1,0,0}}, {1.5,1}, 20, 10000 };
	PointInfluenceArea pointInfluenceArea1{ Pov{{0,.25,-2}, {0,0,1}, 30, 30}, 4, 0.5, 10000 };
	PointInfluenceArea pointInfluenceArea2{ Pov{{0,0,0}, {1,1,0}, 90, 60}, 8, 0.5, 100 };

	//build the ray casters (will later be used to cast rays)
	PlaneRayCaster planeRayCaster1{ planeInfluenceArea1 }; planeRayCaster1.generateRays(rng, 1000, true);
	PlaneRayCaster planeRayCaster2{ planeInfluenceArea2 }; planeRayCaster2.generateRays(rng, 1000, true);
	PointRayCaster pointRayCaster1{ pointInfluenceArea1 }; pointRayCaster1.generateRays(rng, 1000, true);
	PointRayCaster pointRayCaster2{ pointInfluenceArea2 }; pointRayCaster2.generateRays(rng, 1000, true);

	//create BVHs
	Bvh::Properties bvhProperties{};
	bvhProperties.maxLevels = 100;
	bvhProperties.maxLeafCost = 0.01f;
	bvhProperties.maxTrianglesPerLeaf = 2;
	bvhProperties.bins = 40;
	bvhProperties.maxChildrenFatherHitProbabilityRatio = 1.3f;
	bvhProperties.splitPlaneQualityThreshold = 0.1f;

	Bvh baseBvh{ bvhProperties, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesLongest<0.5f>, bvhStrategies::shouldStopThresholdOrLevel, "fallback"};
	Bvh bvh1{ bvhProperties, planeInfluenceArea1, bvhStrategies::computeCostPah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane1"};
	Bvh bvh2{ bvhProperties, planeInfluenceArea2, bvhStrategies::computeCostPah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "plane2"};
	Bvh bvh3{ bvhProperties, pointInfluenceArea1, bvhStrategies::computeCostPah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "point1"};
	Bvh bvh4{ bvhProperties, pointInfluenceArea2, bvhStrategies::computeCostPah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel, "point2"};
	
	//this is the BVH we use with SAH, to compare results
	Bvh sahBvh{ bvhProperties, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesLongest<0.5f>, bvhStrategies::shouldStopThresholdOrLevel, "base SAH"};
	sahBvh.build(triangles);

	//build top level structure
	TopLevel::Properties topLevelProperties{};
	topLevelProperties.maxChildrenFatherHitProbabilityRatio = 1.3;
	topLevelProperties.splitPlaneQualityThreshold = 0.1f;

	vector rayCasters{ &planeRayCaster2/*, &planeRayCaster2 */ };
	TopLevelOctree topLevelStructure{ topLevelProperties, TopLevelOctree::OctreeProperties{ 4, false }, std::move(baseBvh), std::move(bvh2)/*, std::move(bvh2)*/ };
	topLevelStructure.build(triangles);

	//analyze BVH
	TopLevelOctreeAnalyzer analyzer {
		//MAKE_ACTIONS_PAIR(core),
		MAKE_ACTIONS_PAIR(sah),
		MAKE_ACTIONS_PAIR(pah),
		MAKE_ACTIONS_PAIR(levelCount),
		//MAKE_ACTIONS_PAIR(triangles),
		MAKE_ACTIONS_PAIR(influenceArea),
		//MAKE_ACTIONS_PAIR(timeMeasurement)
	};

	auto testScene = TestScene{ "D:/Users/lapof/Documents/Development/ProjectedAreaHeuristic/Results", 
		triangles, &planeRayCaster1, topLevelStructure, analyzer };



	return 0;
	json analysis = analyzer.analyze(topLevelStructure, "D:/Users/lapof/Documents/Development/ProjectedAreaHeuristicVisualizer/Assets/Data/bvh.json");
	
	auto pahTraversalResults = ranges::fold_left(rayCasters, CumulativeRayCasterResults{}, [&topLevelStructure](auto res, auto rayCaster) { return res + rayCaster->castRays(topLevelStructure); });
	auto sahTraversalResults = ranges::fold_left(rayCasters, CumulativeRayCasterResults{}, [&sahBvh](auto res, auto rayCaster) { return res + rayCaster->castRays(sahBvh); });
	
	ofstream pahFile{ "D:/Users/lapof/Documents/Development/ProjectedAreaHeuristic/Results/pah.json" };
	pahFile << setw(2) << json(pahTraversalResults);
	pahFile.close();

	ofstream sahFile{ "D:/Users/lapof/Documents/Development/ProjectedAreaHeuristic/Results/sah.json" };
	sahFile << setw(2) << json(sahTraversalResults);
	sahFile.close();

	return 0;
}
