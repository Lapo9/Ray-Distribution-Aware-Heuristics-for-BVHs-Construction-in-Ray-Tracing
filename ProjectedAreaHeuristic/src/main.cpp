#include "Bvh.h"
#include "InfluenceArea.h"
#include "Utilities.h"
#include "BvhAnalyzer.h"
#include "TopLevel.h"
#include "TopLevelAnalyzer.h"
#include "AnalyzerActions.h"

#include <iostream>
#include <memory>

using namespace std;
using namespace pah;
using namespace nlohmann;

int main() {
	auto M = projection::computeViewMatrix(Pov{ Vector3{9,8,7}, Vector3{1,0,0.5f} });
	//generate triangles
	mt19937 rng{ 1 };
	Uniform3dDistribution mainDistribution3d{ 0,10, 0,10, 0,10 };
	Uniform3dDistribution otherDistribution3d{ -1,1, -1,1 , -1,1 };
	auto triangles = Triangle::generateRandom(100, rng, mainDistribution3d, otherDistribution3d);

	//create influence areas
	PlaneInfluenceArea planeInfluenceArea1{ Plane{Vector3{1,4,0}, Vector3{0,0,1}}, Vector2{1,4}, 20, 10 };
	PlaneInfluenceArea planeInfluenceArea2{ Plane{Vector3{0,2,1}, Vector3{1,0,1}}, Vector2{1,2}, 15, 10 };
	PointInfluenceArea pointInfluenceArea1{ Pov{Vector3{1,1,1}, Vector3{1,0,0}}, 10, 1, 90, 45, 10 };
	PointInfluenceArea pointInfluenceArea2{ Pov{Vector3{0,0,0}, Vector3{1,1,0}}, 8, 0.5, 90, 60, 10 };

	//build BVHs
	Bvh::Properties properties{};
	properties.maxLevels = 100;
	properties.maxLeafCost = 0.1f;
	properties.maxTrianglesPerLeaf = 2;
	properties.bins = 20;

	Bvh bvh1{ properties, planeInfluenceArea1, bvhStrategies::computeCostPah, bvhStrategies::chooseSplittingPlanesFacing<1.0f>, bvhStrategies::shouldStopThresholdOrLevel };
	Bvh bvh2{ properties, planeInfluenceArea2, bvhStrategies::computeCostPah, bvhStrategies::chooseSplittingPlanesFacing<1.0f>, bvhStrategies::shouldStopThresholdOrLevel };
	Bvh bvh3{ properties, pointInfluenceArea1, bvhStrategies::computeCostPah, bvhStrategies::chooseSplittingPlanesFacing<1.0f>, bvhStrategies::shouldStopThresholdOrLevel };
	Bvh bvh4{ properties, pointInfluenceArea2, bvhStrategies::computeCostPah, bvhStrategies::chooseSplittingPlanesFacing<1.0f>, bvhStrategies::shouldStopThresholdOrLevel };

	//build top level structure
	TopLevelOctree topLevelStructure{ TopLevelOctree::Properties{ 4, false }, triangles, std::move(bvh1), std::move(bvh3) };
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

	//std::cout << std::setw(2) << analysis;

	return 0;
}
