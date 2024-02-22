#include "Bvh.h"

#include <limits>
#include <chrono>
#include <queue>

using namespace std;
using namespace pah::utilities;


// ======| Bvh |======
pah::Bvh::Bvh(const Properties& properties, const InfluenceArea& influenceArea, ComputeCostType computeCost, ChooseSplittingPlanesType chooseSplittingPlanes, ShouldStopType shouldStop)
	: properties{ properties }, influenceArea{ &influenceArea }, computeCost{ computeCost }, chooseSplittingPlanes{ chooseSplittingPlanes }, shouldStop{ shouldStop } {
}

pah::Bvh::Bvh(const Properties& properties, ComputeCostType computeCost, ChooseSplittingPlanesType chooseSplittingPlanes, ShouldStopType shouldStop)
	: properties{ properties }, influenceArea{ nullptr }, computeCost{ computeCost }, chooseSplittingPlanes{ chooseSplittingPlanes }, shouldStop{ shouldStop } {
}

void pah::Bvh::build(const std::vector<const Triangle*>& triangles, float splitPlaneQualityThreshold, float maxChildrenFatherHitProbabilityRatio) {
	//the final action simply adds the measured time to the total time
	INFO(TimeLogger timeLogger{ [this](DurationMs duration) { totalBuildTime = duration; } };);

	random_device randomDevice;
	build(triangles, randomDevice(), splitPlaneQualityThreshold, maxChildrenFatherHitProbabilityRatio); //the seed is random
	INFO(timeLogger.stop(););
	//here timeLogger will be destroyed, and it will log (by calling finalAction)
}

void pah::Bvh::build(const std::vector<const Triangle*>& triangles, unsigned int seed, float splitPlaneQualityThreshold, float maxChildrenFatherHitProbabilityRatio) {
	id = chrono::duration_cast<std::chrono::milliseconds>(chrono::system_clock::now().time_since_epoch()).count(); //set the id based on current time: the id is just used to check for equality betweeen 2 BVHs (and this is the only non const function)
	rng = mt19937{ seed }; //initialize random number generator
	//from an array of triangles, to an array to pointers
	root = { triangles }; //initizalize root
	rootMetric = computeCost(root, *influenceArea, -1).cost; //initialize the root metric (generally its area/projected area)
	splitNode(root, Axis::X, std::numeric_limits<float>::max(), 1, splitPlaneQualityThreshold, maxChildrenFatherHitProbabilityRatio);
}

pah::Bvh::TraversalResults pah::Bvh::traverse(const Ray& ray) const {	
	TraversalResults res{};
	TIME(TimeLogger timeLogger{ [&res](auto duration) {res.traversalTime = duration; } });

	queue<const Node*> toVisit{};
	toVisit.push(&root);
	float closestHit = numeric_limits<float>::max();

	while (toVisit.size() > 0) {
		const Node& current = *toVisit.front();
		toVisit.pop(); //queue::front doesn't remove the element from the queue, it just accesses it
		//we enter the if statement iff there is a hit with the box and this hit is closer than the closest hit found so far
		if (const auto& boxHitInfo = collisionDetection::areColliding(ray, current.aabb); boxHitInfo.hit && boxHitInfo.distance < closestHit) {
			res.intersectionTestsTotal++;
			res.intersectionTestsWithNodes++;
			if (current.isLeaf()) {
				const auto& triangles = current.triangles;
				res.traversalCost += LEAF_COST * triangles.size();
				for (const Triangle* triangle : triangles) {
					res.intersectionTestsTotal++;
					res.intersectionTestsWithTriangles++;
					const auto& hitInfo = collisionDetection::areColliding(ray, *triangle);
					if (hitInfo.hit && hitInfo.distance < closestHit) {
						closestHit = hitInfo.distance;
						res.closestHit = triangle;
						res.closestHitDistance = hitInfo.distance;
					}
				}
			}
			else {
				res.traversalCost += NODE_COST * 2.0f;
				toVisit.push(&*current.leftChild);
				toVisit.push(&*current.rightChild);
			}
		}
	}

	INFO(timeLogger.stop(););
	return res;
}

void pah::Bvh::splitNode(Node& node, Axis fatherSplittingAxis, float fatherHitProbability, int currentLevel, float splitPlaneQualityThreshold, float maxChildrenFatherHitProbabilityRatio) {
	//the final action simply adds the measured time to the total time
	TIME(TimeLogger timeLoggerTotal{ [&timingInfo = node.nodeTimingInfo](DurationMs duration) { timingInfo.logTotal(duration); } };);

	float bestLeftCostSoFar = numeric_limits<float>::max(), bestRightCostSoFar = numeric_limits<float>::max();
	float bestLeftHitProbabilitySoFar = numeric_limits<float>::max(), bestRightHitProbabilitySoFar = numeric_limits<float>::max();
	Node bestLeft{ Aabb::maxAabb() }, bestRight{ Aabb::maxAabb() };
	Axis usedAxis; //we save what axis we actually used
	auto splittingPlanes = chooseSplittingPlanesWrapper(node, *influenceArea, fatherSplittingAxis, rng);

	bool found = false; //flag to check if we found at least one split (maybe all the splits place the triangles on one side, leaving the other one empty)

	//the final action simply adds the measured time to the total split time
	TIME(TimeLogger timeLoggerSplitting{ [&timingInfo = node.nodeTimingInfo](DurationMs duration) { timingInfo.logSplittingTot(duration); } };);
	//try to split for each axis provided by chooseSplittingPlanes
	for (auto [axis, quality] : splittingPlanes) {
		bool forceSah = false; // whether to use the standard SAH strategies in place of the user selected ones
		//if the quality of this split plane is too low... and we haven't already found a satisfactory split, fallback to longest + SAH strategy
		if (quality < splitPlaneQualityThreshold) {
			// ...and we haven't already found a satisfactory split, fallback to longest + SAH strategy (only use the best split plane)
			if ((bestLeftHitProbabilitySoFar + bestRightHitProbabilitySoFar) / fatherHitProbability <= maxChildrenFatherHitProbabilityRatio) break;

			forceSah = true;
			Axis sahAxis = chooseSplittingPlanesWrapper(node, *influenceArea, fatherSplittingAxis, rng, forceSah)[0].first;
			axis = sahAxis;
			bestLeftCostSoFar = numeric_limits<float>::max(); bestRightCostSoFar = numeric_limits<float>::max();
		}

		//split for each bin
		for (int i = 1; i < properties.bins - 1; ++i) {
			float splittingPlanePosition = at(node.aabb.min, axis) + (at(node.aabb.max, axis) - at(node.aabb.min, axis)) / properties.bins * i;
			const auto& [leftTriangles, rightTriangles] = splitTriangles(node, node.triangles, axis, splittingPlanePosition);
			if (leftTriangles.size() <= 0 || rightTriangles.size() <= 0) continue; //we must have triangles on both sides to procede

			TIME(TimeLogger timeLoggerNodes{ [&timingInfo = node.nodeTimingInfo](auto duration) { timingInfo.logNodesCreation(duration); } };);
			Node left = { leftTriangles }, right = { rightTriangles };
			TIME(timeLoggerNodes.stop(););

			auto [costLeft, hitProbabilityLeft] = computeCostWrapper(node, left, *influenceArea, rootMetric, forceSah);
			auto [costRight, hitProbabilityRight] = computeCostWrapper(node, right, *influenceArea, rootMetric, forceSah);

			//update best split (also check that we have triangles on both sides, else we might get stuck)
			if (costLeft + costRight < bestLeftCostSoFar + bestRightCostSoFar) {
				found = true;
				usedAxis = axis;
				bestLeftCostSoFar = costLeft;
				bestLeftHitProbabilitySoFar = hitProbabilityLeft;
				bestRightCostSoFar = costRight;
				bestRightHitProbabilitySoFar = hitProbabilityRight;
				bestLeft = std::move(left);
				bestRight = std::move(right);
			}
		}
		if (forceSah) break; //if it was forced to use SAH, it means that the splitting plane quality was low. Therefore it is useless to keep trying (since planes are sorted by their quality).
	}
	TIME(timeLoggerSplitting.stop();); //log the time it took to split this node

	if (!found) return; //if we haven't found at least one split (therefore triangles are not separable), this node is a leaf by definition

	//set children nodes to the best split found (move is necessary, because Node(s) cannot be copied)
	node.leftChild = make_unique<Node>(std::move(bestLeft));
	node.rightChild = make_unique<Node>(std::move(bestRight));

	TIME(timeLoggerTotal.stop();); //log the time it took for this node (of course we exclude recursive calls)

	//recurse on children
	currentLevel++;
	if (!shouldStopWrapper(node, *node.leftChild, properties, currentLevel, bestLeftCostSoFar))
		splitNode(*node.leftChild, usedAxis, bestLeftHitProbabilitySoFar, currentLevel, splitPlaneQualityThreshold, maxChildrenFatherHitProbabilityRatio);
	if (!shouldStopWrapper(node, *node.rightChild, properties, currentLevel, bestRightCostSoFar))
		splitNode(*node.rightChild, usedAxis, bestRightHitProbabilitySoFar, currentLevel, splitPlaneQualityThreshold, maxChildrenFatherHitProbabilityRatio);
}

const pah::Bvh::Node& pah::Bvh::getRoot() const {
	return root;
}

const pah::InfluenceArea* pah::Bvh::getInfluenceArea() const {
	return influenceArea;
}

INFO(const pah::DurationMs pah::Bvh::getTotalBuildTime() const {
	return totalBuildTime;
})

const pah::Bvh::Properties pah::Bvh::getProperties() const {
	return properties;
}

pah::Bvh::ComputeCostReturnType pah::Bvh::computeCostWrapper(const Node& parent, const Node& node, const InfluenceArea& influenceArea, float rootArea, bool forceSah) {
	//the final action simply adds the measured time to the total compute cost time, and increases the compute cost counter
	TIME(TimeLogger timeLogger{ [&timingInfo = parent.nodeTimingInfo](auto duration) { timingInfo.logComputeCost(duration); } };);
	if (forceSah) return bvhStrategies::computeCostSah(node, influenceArea, rootArea);
	return computeCost(node, influenceArea, rootArea);
	//here timeLogger will be destroyed, and it will log (by calling finalAction)
}

pah::Bvh::ChooseSplittingPlanesReturnType pah::Bvh::chooseSplittingPlanesWrapper(const Node& node, const InfluenceArea& influenceArea, Axis axis, mt19937& rng, bool forceSah) {
	//the final action simply adds the measured time to the total choose splitting plane time, and increases the choose splitting plane counter
	TIME(TimeLogger timeLogger{ [&timingInfo = node.nodeTimingInfo](auto duration) { timingInfo.logChooseSplittingPlanes(duration); } };);
	if (forceSah) return bvhStrategies::chooseSplittingPlanesLongest(node, influenceArea, axis, rng);
	return chooseSplittingPlanes(node, influenceArea, axis, rng);
	//here timeLogger will be destroyed, and it will log (by calling finalAction)
}

pah::Bvh::ShouldStopReturnType pah::Bvh::shouldStopWrapper(const Node& parent, const Node& node, const Properties& properties, int currentLevel, float nodeCost, bool forceSah) {
	//the final action simply adds the measured time to the total should stop time, and increases the should stop counter
	TIME(TimeLogger timeLogger{ [&timingInfo = parent.nodeTimingInfo](auto duration) { timingInfo.logShouldStop(duration); } };);
	if (forceSah) return bvhStrategies::shouldStopThresholdOrLevel(node, properties, currentLevel, nodeCost);
	return shouldStop(node, properties, currentLevel, nodeCost);
	//here timeLogger will be destroyed, and it will log (by calling finalAction)
}

