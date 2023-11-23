#include "Bvh.h"
#include "InfluenceArea.h"
#include "Utilities.h"
#include "BvhAnalyzer.h"
#include "AnalyzerActions.h"

#include <iostream>
#include <memory>

using namespace std;
using namespace pah;

int main() {
	Bvh::Properties properties{};
	properties.maxLevels = 100;
	properties.maxLeafCost = 1.0f;
	properties.maxTrianglesPerLeaf = 2;
	properties.bins = 20;

	PlaneInfluenceArea planeInfluenceArea{ Plane{}, Vector2{1,1}, 10 };

	mt19937 rng{ 0 };
	Uniform3dDistribution mainDistribution3d{ 0,10, -5,5, 15,19 };
	Uniform3dDistribution otherDistribution3d{ -1,1, -1,1 , -1,1 };
	auto triangles = Triangle::generateRandom(20, rng, mainDistribution3d, otherDistribution3d);

	Bvh bvh{ properties, planeInfluenceArea, Bvh::computeCostPah, Bvh::chooseSplittingPlanesLongest, Bvh::shouldStopThresholdOrLevel };
	bvh.build(triangles);

	BvhAnalyzer analyzer{ ACTIONS_PAIR(sah), ACTIONS_PAIR(levelCount) };
	
	nlohmann::json analysis = analyzer.analyze(bvh);

	std::cout << std::setw(1) << analysis;
}
