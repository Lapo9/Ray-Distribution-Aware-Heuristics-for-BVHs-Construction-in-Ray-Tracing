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
	//generate triangles
	mt19937 rng{ 1 };
	Uniform3dDistribution mainDistribution3d{ -5,5, -5,5, 2,10 };
	Uniform3dDistribution otherDistribution3d{ -1,1, -1,1 , -1,1 };
	auto triangles = Triangle::generateRandom(100, rng, mainDistribution3d, otherDistribution3d);

	//create influence areas
	PlaneInfluenceArea planeInfluenceArea1{ Plane{}, Vector2{2,10}, 10, 20 };
	PlaneInfluenceArea planeInfluenceArea2{ Plane{Vector3{-5,1,4}, Vector3{1,0,0}}, Vector2{5,1}, 10, 15 };

	//build BVHs
	Bvh::Properties properties{};
	properties.maxLevels = 100;
	properties.maxLeafCost = 0.1f;
	properties.maxTrianglesPerLeaf = 2;
	properties.bins = 20;

	Bvh bvh1{ properties, planeInfluenceArea1, Bvh::computeCostPah, Bvh::chooseSplittingPlanesFacing<1.0f>, Bvh::shouldStopThresholdOrLevel };
	Bvh bvh2{ properties, planeInfluenceArea2, Bvh::computeCostPah, Bvh::chooseSplittingPlanesFacing<1.0f>, Bvh::shouldStopThresholdOrLevel };

	//build top level structure
	TopLevelAabbs topLevelStructure{ triangles, std::move(bvh1), std::move(bvh2) };
	topLevelStructure.build();

	//analyze BVH
	TopLevelAnalyzer analyzer{
		MAKE_ACTIONS_PAIR(core),
		MAKE_ACTIONS_PAIR(sah),
		MAKE_ACTIONS_PAIR(pah),
		MAKE_ACTIONS_PAIR(levelCount),
		MAKE_ACTIONS_PAIR(triangles),
		MAKE_ACTIONS_PAIR(influenceArea),
		MAKE_ACTIONS_PAIR(timeMeasurement)
	};
	json analysis = analyzer.analyze(topLevelStructure, "D:/Users/lapof/Documents/Development/ProjectedAreaHeuristicVisualizer/Assets/Data/bvh.json");

	std::cout << std::setw(2) << analysis;
}
