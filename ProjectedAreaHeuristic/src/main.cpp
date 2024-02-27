#include "Bvh.h"
#include "InfluenceArea.h"
#include "Utilities.h"
#include "BvhAnalyzer.h"
#include "TopLevel.h"
#include "TopLevelAnalyzer.h"
#include "AnalyzerActions.h"
#include "RayCaster.h"
#include "distributions.h"

#include <iostream>
#include <memory>

using namespace std;
using namespace pah;
using namespace nlohmann;

int main() {
	//generate triangles
	mt19937 rng{ 0 };
	distributions::UniformBoxDistribution mainDistribution3d{ 0,10, 0,10, 0,10 };
	distributions::UniformBoxDistribution otherDistribution3d{ -1,1, -1,1 , -1,1 };
	//auto triangles = Triangle::generateRandom(100, rng, mainDistribution3d, otherDistribution3d);
	auto triangles = Triangle::fromObj("D:/Users/lapof/Documents/Development/ProjectedAreaHeuristic/ProjectedAreaHeuristic/exampleData/MonkeyModel.obj");
	auto trianglesSah = triangles;
	auto trianglesSahPtrs = trianglesSah | views::transform([](const Triangle& t) {return &t; }) | ranges::to<vector>();

	//create influence areas
	PlaneInfluenceArea planeInfluenceArea1{ Plane{{-1.5,0,-1.5}, {1,0,1}}, {.5,.5}, 4, 10000 };
	PlaneInfluenceArea planeInfluenceArea2{ Plane{{2,0,0}, {-1,0,0}}, {.5,.5}, 4, 10000 };
	PointInfluenceArea pointInfluenceArea1{ Pov{{0,.25,-2}, {0,0,1}, 30, 30}, 4, 0.5, 10000 };
	PointInfluenceArea pointInfluenceArea2{ Pov{{0,0,0}, {1,1,0}, 90, 60}, 8, 0.5, 100 };

	//build the ray casters (will later be used to cast rays)
	PlaneRayCaster planeRayCaster1{ planeInfluenceArea1 }; planeRayCaster1.generateRays(rng, 10000, true);
	PlaneRayCaster planeRayCaster2{ planeInfluenceArea2 }; planeRayCaster2.generateRays(rng, 10000, true);
	PointRayCaster pointRayCaster1{ pointInfluenceArea1 }; pointRayCaster1.generateRays(rng, 10000, true);
	PointRayCaster pointRayCaster2{ pointInfluenceArea2 }; pointRayCaster2.generateRays(rng, 10000, true);

	//create BVHs
	Bvh::Properties properties{};
	properties.maxLevels = 100;
	properties.maxLeafCost = 0.01f;
	properties.maxTrianglesPerLeaf = 2;
	properties.bins = 40;

	Bvh baseBvh{ properties, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesLongest<0.5f>, bvhStrategies::shouldStopThresholdOrLevel };
	Bvh bvh1{ properties, planeInfluenceArea1, bvhStrategies::computeCostPah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel };
	Bvh bvh2{ properties, planeInfluenceArea2, bvhStrategies::computeCostPah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel };
	Bvh bvh3{ properties, pointInfluenceArea1, bvhStrategies::computeCostPah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel };
	Bvh bvh4{ properties, pointInfluenceArea2, bvhStrategies::computeCostPah, bvhStrategies::chooseSplittingPlanesFacing, bvhStrategies::shouldStopThresholdOrLevel };
	
	//this is the BVH we use with SAH, to compare results
	Bvh sahBvh{ properties, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesLongest<0.5f>, bvhStrategies::shouldStopThresholdOrLevel };
	sahBvh.build(trianglesSahPtrs, 0.1f, 1.3f);

	//build top level structure
	vector<RayCaster<>*> rayCasters{ &pointRayCaster1/*, &planeRayCaster2 */ };
	TopLevelOctree topLevelStructure{ TopLevelOctree::Properties{ 4, false }, triangles, std::move(baseBvh), std::move(bvh3)/*, std::move(bvh2)*/};
	topLevelStructure.build(0.1f, 1.3f);

	//analyze BVH
	TopLevelOctreeAnalyzer analyzer {
		MAKE_ACTIONS_PAIR(core),
		MAKE_ACTIONS_PAIR(sah),
		MAKE_ACTIONS_PAIR(pah),
		MAKE_ACTIONS_PAIR(levelCount),
		MAKE_ACTIONS_PAIR(triangles),
		MAKE_ACTIONS_PAIR(influenceArea),
		MAKE_ACTIONS_PAIR(timeMeasurement)
	};
	json analysis = analyzer.analyze(topLevelStructure, "D:/Users/lapof/Documents/Development/ProjectedAreaHeuristicVisualizer/Assets/Data/bvh.json");
	
	auto pahTraversalResults = ranges::fold_left(rayCasters, CumulativeRayCasterResults{}, [&topLevelStructure](auto res, auto rayCaster) { return res + rayCaster->castRays(topLevelStructure); });
	auto sahTraversalResults = ranges::fold_left(rayCasters, CumulativeRayCasterResults{}, [&sahBvh](auto res, auto rayCaster) { return res + rayCaster->castRays(sahBvh); });
	
	std::ofstream pahFile{ "D:/Users/lapof/Documents/Development/ProjectedAreaHeuristic/Results/pah.json" };
	pahFile << std::setw(2) << json(pahTraversalResults);
	pahFile.close();

	std::ofstream sahFile{ "D:/Users/lapof/Documents/Development/ProjectedAreaHeuristic/Results/sah.json" };
	sahFile << std::setw(2) << json(sahTraversalResults);
	sahFile.close();

	return 0;
}
