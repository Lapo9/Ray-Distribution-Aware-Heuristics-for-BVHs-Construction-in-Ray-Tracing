#include "Bvh.h"
#include <limits>
#include <chrono>

pah::Bvh::Bvh(const Properties& properties, const InfluenceArea& influenceArea, const ComputeCostType& computeCost, const ChooseSplittingPlanesType& chooseSplittingPlanes, const ShouldStopType& shouldStop)
	: properties{ properties }, influenceArea{ make_unique<InfluenceArea>(influenceArea) }, computeCost{ computeCost }, chooseSplittingPlanes{ chooseSplittingPlanes }, shouldStop{ shouldStop } {
}

void pah::Bvh::build(const vector<Triangle*>& triangles) {
	random_device randomDevice;
	build(triangles, randomDevice()); //the seed is random
}

void pah::Bvh::build(const vector<Triangle*>& triangles, unsigned int seed) {
	rng = mt19937{ seed }; //initialize random number generator
	root = { Aabb{triangles} };
}

void pah::Bvh::splitNode(Node& node, Axis fatherSplittingAxis, int currentLevel) {
	float bestLeftSoFar = numeric_limits<float>::max(), bestRightSoFar = numeric_limits<float>::max();
	Node bestLeft{ Aabb::maxAabb() }, bestRight{ Aabb::maxAabb() };
	Axis usedAxis; //we save what axis we actually used
	auto splittingPlanes = chooseSplittingPlanes(node.aabb, *influenceArea, fatherSplittingAxis, rng);

	//try to split for each axis provided by chooseSplittingPlanes
	for (const auto& [axis, criterium] : splittingPlanes) {
		if (!criterium(bestLeftSoFar + bestRightSoFar)) break; //is it worth it to try this split?
		//split for each bin
		for (int i = 1; i < properties.bins - 1; ++i) {
			float splittingPlanePosition = at(node.aabb.min, axis) + (at(node.aabb.max, axis) - at(node.aabb.min, axis)) / properties.bins * i;
			const auto& [leftTriangles, rightTriangles] = splitTriangles(node.triangles, axis, splittingPlanePosition);
			Node left = { leftTriangles }, right = { rightTriangles };
			float costLeft = computeCost(left), costRight = computeCost(right);

			//update best split
			if (costLeft + costRight < bestLeftSoFar + bestRightSoFar) {
				usedAxis = axis;
				bestLeftSoFar = costLeft;
				bestRightSoFar = costRight;
				bestLeft = std::move(left);
				bestRight = std::move(right);
			}
		}
	}

	//set children nodes to the best split found
	node.leftChild = std::make_unique<Node>(bestLeft);
	node.rightChild = std::make_unique<Node>(bestRight);

	//recurse on children
	currentLevel++;
	if (!shouldStop(properties, *node.leftChild, currentLevel, bestLeftSoFar)) splitNode(*node.leftChild, usedAxis);
	if (!shouldStop(properties, *node.rightChild, currentLevel, bestRightSoFar)) splitNode(*node.rightChild, usedAxis);
}

const pah::Bvh::Node& pah::Bvh::getRoot() const {
	return root;
}
