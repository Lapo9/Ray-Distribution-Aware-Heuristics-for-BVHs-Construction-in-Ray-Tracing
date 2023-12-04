#include "Bvh.h"
#include <limits>
#include <chrono>
#include <algorithm>

using namespace std;
using namespace pah::utilities;

pah::Bvh::Bvh(const Properties& properties, const InfluenceArea& influenceArea, ComputeCostType computeCost, ChooseSplittingPlanesType chooseSplittingPlanes, ShouldStopType shouldStop)
	: properties{ properties }, influenceArea{ &influenceArea }, computeCost{ computeCost }, chooseSplittingPlanes{ chooseSplittingPlanes }, shouldStop{ shouldStop } {
	//note that we MUST instantiate the unique_ptr this way (a.k.a. we cannot use make_unique), because make_unique tries to allocate the base class, which is abstract
}

void pah::Bvh::build(const vector<Triangle>& triangles) {
	random_device randomDevice;
	build(triangles, randomDevice()); //the seed is random
}

void pah::Bvh::build(const vector<Triangle>& triangles, unsigned int seed) {
	id = chrono::duration_cast<std::chrono::milliseconds>(chrono::system_clock::now().time_since_epoch()).count(); //set the id based on current time: the id is just used to check for equality betweeen 2 BVHs (and this is the only non const function)
	rng = mt19937{ seed }; //initialize random number generator
	//from an array of triangles, to an array to pointers
	vector<const Triangle*> trianglesPointers(triangles.size()); std::transform(triangles.begin(), triangles.end(), trianglesPointers.begin(), [](const Triangle& t) { return &t; });
	root = { trianglesPointers }; //initizalize root
	rootMetric = computeCost(root, *influenceArea, -1); //initialize the root metric (generally its area/projected area)
	splitNode(root, Axis::X, 1);
}

void pah::Bvh::splitNode(Node& node, Axis fatherSplittingAxis, int currentLevel) {
	float bestLeftSoFar = numeric_limits<float>::max(), bestRightSoFar = numeric_limits<float>::max();
	Node bestLeft{ Aabb::maxAabb() }, bestRight{ Aabb::maxAabb() };
	Axis usedAxis; //we save what axis we actually used
	auto splittingPlanes = chooseSplittingPlanes(node.aabb, *influenceArea, fatherSplittingAxis, rng);

	bool found = false; //flag to check if we found at least one split (maybe all the splits place the triangles on one side, leaving the other one empty)
	//try to split for each axis provided by chooseSplittingPlanes
	for (const auto& [axis, criterium] : splittingPlanes) {
		if (found && !criterium(bestLeftSoFar + bestRightSoFar)) break; //is it worth it to try this split? (it is always worth it if we haven't found a split on the previou axis)
		//split for each bin
		for (int i = 1; i < properties.bins - 1; ++i) {
			float splittingPlanePosition = at(node.aabb.min, axis) + (at(node.aabb.max, axis) - at(node.aabb.min, axis)) / properties.bins * i;
			const auto& [leftTriangles, rightTriangles] = splitTriangles(node.triangles, axis, splittingPlanePosition);
			Node left = { leftTriangles }, right = { rightTriangles };
			float costLeft = computeCost(left, *influenceArea, rootMetric), costRight = computeCost(right, *influenceArea, rootMetric);

			//update best split (also check that we have triangles on both sides, else we might get stuck)
			if (costLeft + costRight < bestLeftSoFar + bestRightSoFar && leftTriangles.size() > 0 && rightTriangles.size() > 0) {
				found = true;
				usedAxis = axis;
				bestLeftSoFar = costLeft;
				bestRightSoFar = costRight;
				bestLeft = std::move(left);
				bestRight = std::move(right);
			}
		}
	}

	if (!found) return; //if we haven't found at least one split (therefore triangles are not separable), this node is a leaf by definition

	//set children nodes to the best split found (move is necessary, because Node(s) cannot be copied)
	node.leftChild = std::make_unique<Node>(std::move(bestLeft));
	node.rightChild = std::make_unique<Node>(std::move(bestRight));

	//recurse on children
	currentLevel++;
	if (!shouldStop(properties, *node.leftChild, currentLevel, bestLeftSoFar)) splitNode(*node.leftChild, usedAxis, currentLevel);
	if (!shouldStop(properties, *node.rightChild, currentLevel, bestRightSoFar)) splitNode(*node.rightChild, usedAxis, currentLevel);
}

const pah::Bvh::Node& pah::Bvh::getRoot() const {
	return root;
}

const pah::InfluenceArea& pah::Bvh::getInfluenceArea() const {
	return *influenceArea;
}

pah::Bvh::ComputeCostReturnType pah::Bvh::computeCostWrapper(const Node& node, const InfluenceArea& influenceArea, float rootArea) {
	TimeLogger timeLogger{ std::bind(&NodeTimingInfo::logComputeCost, &(node.nodeTimingInfo), placeholders::_1) };
	auto result = computeCost(node, influenceArea, rootArea);
	DBG()
	return result;
}

pah::Bvh::ChooseSplittingPlanesReturnType pah::Bvh::chooseSplittingPlaneWrapper(const Aabb& aabb, const InfluenceArea& influenceArea, Axis axis, mt19937& rng) {
	auto result = chooseSplittingPlanes(aabb, influenceArea, axis, rng);
	return result;
}

pah::Bvh::ShouldStopReturnType pah::Bvh::shouldStopWrapper(const Properties& properties, const Node& node, int currentLevel, float nodeCost) {
	auto result = shouldStop(properties, node, currentLevel, nodeCost);
	return result;
}
