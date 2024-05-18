/** @file */
#pragma once

#include "../libs/json.hpp"
#include "Bvh.h"
#include "Utilities.h"


#define ANALYZER_ACTION_PER_NODE_ARGUMENTS const Bvh::Node& node, const Bvh& bvh, int currentLevel, json& localLog /**< The arguments (apart the first one, which is specific to each function) that must be part of the per-node function signatures. */
#define ANALYZER_ACTION_FINAL_ARGUMENTS const Bvh& bvh, json& log /**< The arguments (apart the first one, which is specific to each function) that must be part of the final function signatures. */

/**
 * @brief Contains a set of functions that can be used as actions for a @p BvhAnalyzer.
 */
namespace pah::analyzerActions {

	/** Creates a pair of analyzer actions based on name (if an action with such name doesn't exist it fails at compile-time. */
	#define MAKE_ACTIONS_PAIR(actionName) std::pair{ std::function{ pah::analyzerActions::perNode::actionName }, std::function{ pah::analyzerActions::finals::actionName } } 
	
	/** Creates an argument list of analyzer actions based on name (if an action with such name doesn't exist it fails at compile-time. */
	#define MAKE_ACTIONS_ARGS(actionName) std::function{ pah::analyzerActions::perNode::actionName }, std::function{ pah::analyzerActions::finals::actionName }

	/**
	 * @brief Actions executed once for each node.
	 */
	namespace perNode {
		/**
		 * @brief Always adds one node, if the node is also a leaf, adds a leaf too. Adds core info about the node.
		 *
		 * @param nodesAndLeaves Pair containing total number of nodes and leaves so far.
		 */
		static void core(std::pair<int, int>& nodesAndLeaves, ANALYZER_ACTION_PER_NODE_ARGUMENTS) {
			nodesAndLeaves.first++;
			nodesAndLeaves.second += node.isLeaf();

			localLog["core"] = node;
		}

		/**
		 * @brief Calculates the SAH (and surface area) for the node, and adds it to the JSON. Moreover it updates the total SAH, based on the fact that the node is a leaf or internal.
		 * TotalSAH = summation[leaves](prob(i) * costTriangle * #triangles(i)) + summation[internals](prob(i) * costNode * 2)
		 * where 2 is the number of children we must visit if an internal node is hit, and prob(i) = surfaceArea(i) / surfaceArea(root)
		 */
		static void sah(float& totalSah, ANALYZER_ACTION_PER_NODE_ARGUMENTS) {
			float rootSa = bvh.getRoot().aabb.surfaceArea();
			auto [sah, hitProb, sa] = bvhStrategies::computeCostSah(node, bvh.getInfluenceArea(), rootSa);

			//add to JSON
			localLog["metrics"]["sah"] = sah;
			localLog["metrics"]["sa"] = sa;

			//update global variable
			if (node.isLeaf()) { totalSah += sah; } //if a node is a leaf, its cost is actually the SAH: hitProbability * (costTriangle * #triangles)
			else { totalSah += hitProb * NODE_COST * 2.0f; } //if a node is internal, its cost is: hitProbability * (costNode * 2) where 2 is the number of children we now must visit
		}

		/**
		 * @brief Calculates the PAH (and projected area) for the node, and adds it to the JSON. Moreover it updates the total PAH, based on the fact that the node is a leaf or internal.
		 * TotalPAH = summation[leaves](prob(i) * costTriangle * #triangles(i)) + summation[internals](prob(i) * costNode * 2)
		 * where 2 is the number of children we must visit if an internal node is hit, and prob(i) = projectedArea(i) / projectedArea(root)
		 */
		static void pah(float& totalPah, ANALYZER_ACTION_PER_NODE_ARGUMENTS) {
			if (bvh.getInfluenceArea() == nullptr) return; //it means it is a SAH BVH
			//TODO float fullProjectionArea = bvh.getInfluenceArea()->getProjectedArea(bvh.getRoot().aabb);
			float fullProjectionArea = bvh.getInfluenceArea()->getProjectionPlaneArea();
			
			auto [pah, hitProb, pa] = PAH_STRATEGY(node, bvh.getInfluenceArea(), fullProjectionArea);

			//add to JSON
			localLog["metrics"]["pah"] = pah;
			localLog["metrics"]["pa"] = pa;

			//update global variable
			if (node.isLeaf()) { totalPah += pah; } //if a node is a leaf, its cost is actually the PAH: hitProbability * (costTriangle * #triangles)
			else { totalPah += hitProb * NODE_COST * 2.0f; } //if a node is internal, its cost is: hitProbability * (costNode * 2) where 2 is the number of children we now must visit
		}

		/**
		 * @brief Updates the maximum level of the BVH.
		 */
		static void levelCount(int& maxLevel, ANALYZER_ACTION_PER_NODE_ARGUMENTS) {
			maxLevel = currentLevel > maxLevel ? currentLevel : maxLevel;
		}

		/**
		 * @brief Adds all triangles to the list \p triangles. Basically it adds only the triangles of the leaves, since their union is the full set of triangles.
		 */
		static void triangles(std::vector<const Triangle*>& triangles, ANALYZER_ACTION_PER_NODE_ARGUMENTS) {
			if (node.isLeaf()) {
				for (auto& triangle : node.triangles) {
					triangles.push_back(triangle);
				}
			}
		}

		/**
		 * @brief Actually does nothing, we just need the final action.
		 */
		static void influenceArea(int& unused, ANALYZER_ACTION_PER_NODE_ARGUMENTS) {
			//do nothing
		}

		/**
		 * @brief Logs time measurements and updates the total "average" time measurement.
		 * If the macro TIMING is false, nothing happens.
		 */
		static void timeMeasurement(std::pair<Bvh::NodeTimingInfo, int>& meanTimeInfo, ANALYZER_ACTION_PER_NODE_ARGUMENTS) {
			//update total "average" times
			TIME(meanTimeInfo.first += node.nodeTimingInfo;);
			TIME(meanTimeInfo.second += node.isLeaf() ? 0 : 1;);
			
			//log to JSON the times of this node
			TIME(localLog["timing"] = node.nodeTimingInfo;);
		}

		/**
		 * @brief Computes the percentage of the overlapping area between the 2 children.
		 */
		template<int maxLevelToConsider>
		static void siblingsOverlapping(std::tuple<float,float,float,float>& totalAndOverlappingArea, ANALYZER_ACTION_PER_NODE_ARGUMENTS) {
			if (node.isLeaf() || !bvh.getInfluenceArea() || currentLevel > maxLevelToConsider) return;

			const auto& contourPointsLeft = ConvexHull2d{ bvh.getInfluenceArea()->getProjectedHull(node.leftChild->aabb) };
			const auto& contourPointsRight = ConvexHull2d{ bvh.getInfluenceArea()->getProjectedHull(node.rightChild->aabb) };
			auto overlappingChildrenHull = overlappingHull(contourPointsLeft, contourPointsRight);
			float overlappingChildrenArea = overlappingChildrenHull.computeArea();
			float smallestChildrenArea = glm::min(contourPointsLeft.computeArea(), contourPointsRight.computeArea());

			// compute culled areas
			auto projectionPlaneHull = ConvexHull2d{ bvh.getInfluenceArea()->getProjectionPlaneHull() };
			const auto& contourPointsLeftCulled = overlappingHull(contourPointsLeft, projectionPlaneHull);
			const auto& contourPointsRightCulled = overlappingHull(contourPointsRight, projectionPlaneHull);
			auto overlappingChildrenHullCulled = overlappingHull(contourPointsLeftCulled, contourPointsRightCulled);
			float overlappingChildrenAreaCulled = overlappingChildrenHullCulled.computeArea();
			float smallestChildrenAreaCulled = glm::min(contourPointsLeftCulled.computeArea(), contourPointsRightCulled.computeArea());
			
			get<0>(totalAndOverlappingArea) += smallestChildrenArea;
			get<1>(totalAndOverlappingArea) += overlappingChildrenArea;
			get<2>(totalAndOverlappingArea) += smallestChildrenAreaCulled;
			get<3>(totalAndOverlappingArea) += overlappingChildrenAreaCulled;

			float overlappingPercentage = smallestChildrenArea == 0 ? 0.f : overlappingChildrenArea / smallestChildrenArea;
			float overlappingPercentageCulled = smallestChildrenAreaCulled == 0 ? 0.f : overlappingChildrenAreaCulled / smallestChildrenAreaCulled;
			localLog["metrics"]["childrenOverlappingPercentage"+maxLevelToConsider] = overlappingPercentage;
			localLog["metrics"]["childrenOverlappingPercentageCulled"+maxLevelToConsider] = overlappingPercentageCulled;
		}
	}


	/**
	 * @brief Actions executed at the end of the analysis.
	 */
	namespace finals {
		/**
		 * @brief Simply adds the number of nodes and leaves to the JSON.
		 */
		static void core(std::pair<int, int>& nodesAndLeaves, ANALYZER_ACTION_FINAL_ARGUMENTS) {
			log["globalInfo"]["numberOfNodes"] = nodesAndLeaves.first;
			log["globalInfo"]["numberOfLeaves"] = nodesAndLeaves.second;
			log["globalInfo"]["properties"] = bvh.getProperties();
			log["globalInfo"]["name"] = bvh.name;
		}

		/**
		 * @brief Simply adds the total SAH cost to the JSON.
		 */
		static void sah(float& totalSah, ANALYZER_ACTION_FINAL_ARGUMENTS) {
			log["globalInfo"]["sahCost"] = totalSah;
		}

		/**
		 * @brief Simply adds the total PAH cost to the JSON.
		 */
		static void pah(float& totalPah, ANALYZER_ACTION_FINAL_ARGUMENTS) {
			log["globalInfo"]["pahCost"] = totalPah;
		}

		/**
		 * @brief Simply adds the max level to the JSON.
		 */
		static void levelCount(int& maxLevel, ANALYZER_ACTION_FINAL_ARGUMENTS) {
			log["globalInfo"]["maxLevel"] = maxLevel;
		}

		/**
		 * @brief Creates a list of all triangles in \p triangles.
		 */
		static void triangles(std::vector<const Triangle*>& triangles, ANALYZER_ACTION_FINAL_ARGUMENTS) {
			for (auto& triangle : triangles) {
				log["triangles"] += *triangle;
			}
		}

		/**
		 * @brief Adds the influence area to the JSON.
		 */
		static void influenceArea(int& unused, ANALYZER_ACTION_FINAL_ARGUMENTS) {
			if (bvh.getInfluenceArea() != nullptr) {
				log["influenceArea"] = *bvh.getInfluenceArea();
			}
		}

		/**
		 * @brief Logs the total build time and the average times for each building step of each node.
		 */
		static void timeMeasurement(std::pair<Bvh::NodeTimingInfo, int>& meanTimeInfo, ANALYZER_ACTION_FINAL_ARGUMENTS) {			
			//log total "average" times
			TIME(log["totalTiming"] = meanTimeInfo.first;);

			//per node averages (above is per function call)
			TIME(log["totalTiming"]["computeCostCountPerNode"] = (float)meanTimeInfo.first.computeCostCount / meanTimeInfo.second;)
			TIME(log["totalTiming"]["chooseSplittingPlanesCountPerNode"] = (float)meanTimeInfo.first.chooseSplittingPlanesCount / meanTimeInfo.second;)
			TIME(log["totalTiming"]["shouldStopCountPerNode"] = (float)meanTimeInfo.first.shouldStopCount / meanTimeInfo.second;)

			//log total time
			INFO(log["totalTiming"]["fullTotal"] = bvh.getTotalBuildTime().count(););
		}

		/**
		 * @brief Computes the total percentage of overlapping area among siblings.
		 */
		static void siblingsOverlapping(std::tuple<float,float,float,float>& totalAndOverlappingArea, ANALYZER_ACTION_FINAL_ARGUMENTS) {
			log["globalInfo"]["siblingsOverlappingPercentage"] = get<1>(totalAndOverlappingArea) / get<0>(totalAndOverlappingArea);
			log["globalInfo"]["siblingsOverlappingPercentageCulled"] = get<3>(totalAndOverlappingArea) / get<2>(totalAndOverlappingArea);
		}
	}
}
