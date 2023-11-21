#include "Bvh.h"
#include "InfluenceArea.h"
#include "Utilities.h"

#include <iostream>

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

	Bvh bvh{ properties, planeInfluenceArea, Bvh::computeCostSah, Bvh::chooseSplittingPlanesLongest, Bvh::shouldStopThresholdOrLevel };
	bvh.build(triangles);

	std::cout << "End";
}
