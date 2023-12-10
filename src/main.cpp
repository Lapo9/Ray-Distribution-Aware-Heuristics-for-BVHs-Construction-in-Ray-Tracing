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

	//create influence area
	PlaneInfluenceArea planeInfluenceArea{ Plane{}, Vector2{10,10}, 10, Vector3{20,10,40} };

	//build BVH
	Bvh::Properties properties{};
	properties.maxLevels = 100;
	properties.maxLeafCost = 0.1f;
	properties.maxTrianglesPerLeaf = 2;
	properties.bins = 20;
	Bvh bvh{ properties, planeInfluenceArea, Bvh::computeCostPah, Bvh::chooseSplittingPlanesFacing<1.0f>, Bvh::shouldStopThresholdOrLevel };

	//build top level structure
	TopLevelAabbs topLevelStructure{ triangles, std::move(bvh) };
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
