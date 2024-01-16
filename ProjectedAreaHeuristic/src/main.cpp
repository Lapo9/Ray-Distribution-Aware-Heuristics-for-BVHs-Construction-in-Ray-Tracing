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
	mt19937 rng{ 1 };
	distributions::UniformBoxDistribution mainDistribution3d{ 0,10, 0,10, 0,10 };
	distributions::UniformBoxDistribution otherDistribution3d{ -1,1, -1,1 , -1,1 };
	auto triangles = Triangle::generateRandom(100, rng, mainDistribution3d, otherDistribution3d);

	//create influence areas
	PlaneInfluenceArea planeInfluenceArea1{ Plane{Vector3{1,4,0}, Vector3{0,0,1}}, Vector2{1,4}, 20, 100 };
	PlaneInfluenceArea planeInfluenceArea2{ Plane{Vector3{0,2,1}, Vector3{1,0,1}}, Vector2{1,2}, 15, 100 };
	PointInfluenceArea pointInfluenceArea1{ Pov{Vector3{1,1,1}, Vector3{1,0,0}, 90, 45}, 10, 1, 100 };
	PointInfluenceArea pointInfluenceArea2{ Pov{Vector3{0,0,0}, Vector3{1,1,0}, 90, 60}, 8, 0.5, 100 };

	//build the ray casters (will later be used to cast rays)
	PlaneRayCaster planeRayCaster1{ planeInfluenceArea1 }; planeRayCaster1.generateRays(rng, 100);
	PlaneRayCaster planeRayCaster2{ planeInfluenceArea2 }; planeRayCaster2.generateRays(rng, 100);
	PointRayCaster pointRayCaster1{ pointInfluenceArea1 }; pointRayCaster1.generateRays(rng, 100);
	PointRayCaster pointRayCaster2{ pointInfluenceArea2 }; pointRayCaster2.generateRays(rng, 100);

	//build BVHs
	Bvh::Properties properties{};
	properties.maxLevels = 100;
	properties.maxLeafCost = 0.1f;
	properties.maxTrianglesPerLeaf = 2;
	properties.bins = 20;

	Bvh baseBvh{ properties, bvhStrategies::computeCostSah, bvhStrategies::chooseSplittingPlanesLongest<1.0f>, bvhStrategies::shouldStopThresholdOrLevel };
	Bvh bvh1{ properties, planeInfluenceArea1, bvhStrategies::computeCostPah, bvhStrategies::chooseSplittingPlanesFacing<1.0f>, bvhStrategies::shouldStopThresholdOrLevel };
	Bvh bvh2{ properties, planeInfluenceArea2, bvhStrategies::computeCostPah, bvhStrategies::chooseSplittingPlanesFacing<1.0f>, bvhStrategies::shouldStopThresholdOrLevel };
	Bvh bvh3{ properties, pointInfluenceArea1, bvhStrategies::computeCostPah, bvhStrategies::chooseSplittingPlanesFacing<1.0f>, bvhStrategies::shouldStopThresholdOrLevel };
	Bvh bvh4{ properties, pointInfluenceArea2, bvhStrategies::computeCostPah, bvhStrategies::chooseSplittingPlanesFacing<1.0f>, bvhStrategies::shouldStopThresholdOrLevel };

	//build top level structure
	vector<RayCaster<>*> rayCasters{ &planeRayCaster1, &pointRayCaster1 };
	TopLevelOctree topLevelStructure{ TopLevelOctree::Properties{ 4, false }, triangles, std::move(baseBvh), std::move(bvh1), std::move(bvh3) };
	topLevelStructure.build();

	//analyze BVH
	TopLevelOctreeAnalyzer analyzer{
		MAKE_ACTIONS_PAIR(core),
		MAKE_ACTIONS_PAIR(sah),
		MAKE_ACTIONS_PAIR(pah),
		MAKE_ACTIONS_PAIR(levelCount),
		MAKE_ACTIONS_PAIR(triangles),
		MAKE_ACTIONS_PAIR(influenceArea),
		MAKE_ACTIONS_PAIR(timeMeasurement)
	};
	json analysis = analyzer.analyze(topLevelStructure, "D:/Users/lapof/Documents/Development/ProjectedAreaHeuristicVisualizer/Assets/Data/bvh.json");
	
	auto traversalResults = ranges::fold_left(rayCasters, CumulativeRayCasterResults{}, [&topLevelStructure](auto res, auto rayCaster) { return res + rayCaster->castRays(topLevelStructure); });
	cout << setw(2) << json(traversalResults) << endl;

	return 0;
}
