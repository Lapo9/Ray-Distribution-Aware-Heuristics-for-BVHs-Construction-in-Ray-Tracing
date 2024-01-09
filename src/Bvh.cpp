#include "Bvh.h"

#include <limits>
#include <chrono>
#include <queue>

using namespace std;
using namespace pah::utilities;


// ======| Bvh |======
pah::Bvh::Bvh(const Properties& properties, const InfluenceArea& influenceArea, ComputeCostType computeCost, ChooseSplittingPlanesType chooseSplittingPlanes, ShouldStopType shouldStop)
	: properties{ properties }, influenceArea{ &influenceArea }, computeCost{ computeCost }, chooseSplittingPlanes{ chooseSplittingPlanes }, shouldStop{ shouldStop } {}
	
void pah::Bvh::build(const vector<const Triangle*>& triangles) {
	//the final action simply adds the measured time to the total time
	INFO(TimeLogger timeLogger{ [this](NodeTimingInfo::DurationMs duration) { totalBuildTime = duration; } };);

	random_device randomDevice;
	build(triangles, randomDevice()); //the seed is random
	timeLogger.stop();
	//here timeLogger will be destroyed, and it will log (by calling finalAction)
}

void pah::Bvh::build(const vector<const Triangle*>& triangles, unsigned int seed) {
	id = chrono::duration_cast<std::chrono::milliseconds>(chrono::system_clock::now().time_since_epoch()).count(); //set the id based on current time: the id is just used to check for equality betweeen 2 BVHs (and this is the only non const function)
	rng = mt19937{ seed }; //initialize random number generator
	//from an array of triangles, to an array to pointers
	root = { triangles }; //initizalize root
	rootMetric = computeCost(root, *influenceArea, -1); //initialize the root metric (generally its area/projected area)
	splitNode(root, Axis::X, 1);
}

void pah::Bvh::splitNode(Node& node, Axis fatherSplittingAxis, int currentLevel) {
	//the final action simply adds the measured time to the total time
	TIME(TimeLogger timeLoggerTotal{ [&timingInfo = node.nodeTimingInfo](NodeTimingInfo::DurationMs duration) { timingInfo.logTotal(duration); } };);

	float bestLeftSoFar = numeric_limits<float>::max(), bestRightSoFar = numeric_limits<float>::max();
	Node bestLeft{ Aabb::maxAabb() }, bestRight{ Aabb::maxAabb() };
	Axis usedAxis; //we save what axis we actually used
	auto splittingPlanes = chooseSplittingPlanesWrapper(node, *influenceArea, fatherSplittingAxis, rng);

	bool found = false; //flag to check if we found at least one split (maybe all the splits place the triangles on one side, leaving the other one empty)

	//the final action simply adds the measured time to the total split time
	TIME(TimeLogger timeLoggerSplitting{ [&timingInfo = node.nodeTimingInfo](NodeTimingInfo::DurationMs duration) { timingInfo.logSplittingTot(duration); } };);
	//try to split for each axis provided by chooseSplittingPlanes
	for (const auto& [axis, criterium] : splittingPlanes) {
		if (found && !criterium(bestLeftSoFar + bestRightSoFar)) break; //is it worth it to try this split? (it is always worth it if we haven't found a split on the previou axis)
		//split for each bin
		for (int i = 1; i < properties.bins - 1; ++i) {
			float splittingPlanePosition = at(node.aabb.min, axis) + (at(node.aabb.max, axis) - at(node.aabb.min, axis)) / properties.bins * i;
			const auto& [leftTriangles, rightTriangles] = splitTriangles(node, node.triangles, axis, splittingPlanePosition);

			TIME(TimeLogger timeLoggerNodes{ [&timingInfo = node.nodeTimingInfo](auto duration) { timingInfo.logNodesCreation(duration); } };);
			Node left = { leftTriangles }, right = { rightTriangles };
			TIME(timeLoggerNodes.stop(););

			float costLeft = computeCostWrapper(node, left, *influenceArea, rootMetric), costRight = computeCostWrapper(node, right, *influenceArea, rootMetric);

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
	TIME(timeLoggerSplitting.stop();); //log the time it took to split this node

	if (!found) return; //if we haven't found at least one split (therefore triangles are not separable), this node is a leaf by definition

	//set children nodes to the best split found (move is necessary, because Node(s) cannot be copied)
	node.leftChild = make_unique<Node>(std::move(bestLeft));
	node.rightChild = make_unique<Node>(std::move(bestRight));

	TIME(timeLoggerTotal.stop();); //log the time it took for this node (of course we exclude recursive calls)

	//recurse on children
	currentLevel++;
	if (!shouldStopWrapper(node, *node.leftChild, properties, currentLevel, bestLeftSoFar)) splitNode(*node.leftChild, usedAxis, currentLevel);
	if (!shouldStopWrapper(node, *node.rightChild, properties, currentLevel, bestRightSoFar)) splitNode(*node.rightChild, usedAxis, currentLevel);
}

const pah::Bvh::Node& pah::Bvh::getRoot() const {
	return root;
}

const pah::InfluenceArea& pah::Bvh::getInfluenceArea() const {
	return *influenceArea;
}

INFO(const pah::Bvh::NodeTimingInfo::DurationMs pah::Bvh::getTotalBuildTime() const {
	return totalBuildTime;
})

const pah::Bvh::Properties pah::Bvh::getProperties() const {
	return properties;
}

pah::Bvh::ComputeCostReturnType pah::Bvh::computeCostWrapper(const Node& parent, const Node& node, const InfluenceArea& influenceArea, float rootArea) {
	//the final action simply adds the measured time to the total compute cost time, and increases the compute cost counter
	TIME(TimeLogger timeLogger{ [&timingInfo = parent.nodeTimingInfo](auto duration) { timingInfo.logComputeCost(duration); } };);
	return computeCost(node, influenceArea, rootArea);
	//here timeLogger will be destroyed, and it will log (by calling finalAction)
}

pah::Bvh::ChooseSplittingPlanesReturnType pah::Bvh::chooseSplittingPlanesWrapper(const Node& node, const InfluenceArea& influenceArea, Axis axis, mt19937& rng) {
	//the final action simply adds the measured time to the total choose splitting plane time, and increases the choose splitting plane counter
	TIME(TimeLogger timeLogger{ [&timingInfo = node.nodeTimingInfo](auto duration) { timingInfo.logChooseSplittingPlanes(duration); } };);
	return chooseSplittingPlanes(node, influenceArea, axis, rng);
	//here timeLogger will be destroyed, and it will log (by calling finalAction)
}

pah::Bvh::ShouldStopReturnType pah::Bvh::shouldStopWrapper(const Node& parent, const Node& node, const Properties& properties, int currentLevel, float nodeCost) {
	//the final action simply adds the measured time to the total should stop time, and increases the should stop counter
	TIME(TimeLogger timeLogger{ [&timingInfo = parent.nodeTimingInfo](auto duration) { timingInfo.logShouldStop(duration); } };);
	return shouldStop(node, properties, currentLevel, nodeCost);
	//here timeLogger will be destroyed, and it will log (by calling finalAction)
}

pah::Bvh::TraversalResults pah::Bvh::traverse(const Ray& ray) const {
	TraversalResults res{};
	queue<const Node*> toVisit{};
	toVisit.push(&root);

	while (toVisit.size() > 0) {
		const Node& current = *toVisit.front();
		toVisit.pop(); //queue::front doesn't remove the element from the queue, it just accesses it
		if (collisionDetection::areColliding(ray, current.aabb)) {
			res.intersectionsCount++;
			if (current.isLeaf()) {
				res.traversalCost += LEAF_COST * current.triangles.size();
			}
			else {
				res.traversalCost += NODE_COST * 2.0f;
				toVisit.push(&*current.leftChild);
				toVisit.push(&*current.rightChild);
			}
		}
	}
}
