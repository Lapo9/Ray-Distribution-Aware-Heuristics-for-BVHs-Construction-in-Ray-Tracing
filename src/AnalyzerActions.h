/** @file */
#pragma once

#include "../libs/json.hpp"
#include "Bvh.h"
#include "Utilities.h"


#define ANALYZER_ACTION_PER_NODE_ARGUMENTS const Bvh::Node& node, const Bvh& bvh, int currentLevel, json& localLog /**< The arguments (apart the first one, which is specific to each function) that must be part of the per-node function signatures. */
#define ANALYZER_ACTION_FINAL_ARGUMENTS json& log /**< The arguments (apart the first one, which is specific to each function) that must be part of the final function signatures. */


namespace pah::analyzerActions {

	/** Creates a pair of analyzer actions based on name (if an action with such name doesn't exist it fails at compile-time. */
	#define ACTIONS_PAIR(actionName) pair{ function{ pah::analyzerActions::perNode::actionName }, function{ pah::analyzerActions::finals::actionName } } 

	namespace perNode {
		/**
		 * @brief Always adds one node, if the node is also a leaf, adds a leaf too.
		 *
		 * @param nodesAndLeaves Pair containing total number of nodes and leaves so far.
		 */
		static void nodesAndLeaves(pair<int, int>& nodesAndLeaves, ANALYZER_ACTION_PER_NODE_ARGUMENTS) {
			nodesAndLeaves.first++;
			nodesAndLeaves.second += node.isLeaf();
		}

		/**
		 * @brief Calculates the SAH (and surface area) for the node, and adds it to the JSON. Moreover it updates the total SAH, based on the fact that the node is a leaf or internal.
		 * TotalSAH = summation[leaves](prob(i) * costTriangle * #triangles(i)) + summation[internals](prob(i) * costNode * 2)
		 * where 2 is the number of children we must visit if an internal node is hit, and prob(i) = surfaceArea(i) / surfaceArea(root)
		 */
		static void sah(float& totalSah, ANALYZER_ACTION_PER_NODE_ARGUMENTS) {
			float rootSa = bvh.getRoot().aabb.surfaceArea();
			float sah = Bvh::computeCostSah(node, bvh.getInfluenceArea(), rootSa);
			float sa = node.aabb.surfaceArea();

			//add to JSON
			localLog["metrics"]["sah"] = sah;
			localLog["metrics"]["sa"] = sa;

			//update global variable
			if (node.isLeaf()) { totalSah += sah; } //if a node is a leaf, its cost is actually the SAH: hitProbability * (costTriangle * #triangles)
			else { totalSah += (sa / rootSa) * 1.0f * 2.0f; } //if a node is internal, its cost is: hitProbability * (costNode * 2) where 2 is the number of children we now must visit
		}

		/**
		 * @brief Calculates the PAH (and projected area) for the node, and adds it to the JSON. Moreover it updates the total PAH, based on the fact that the node is a leaf or internal.
		 * TotalPAH = summation[leaves](prob(i) * costTriangle * #triangles(i)) + summation[internals](prob(i) * costNode * 2)
		 * where 2 is the number of children we must visit if an internal node is hit, and prob(i) = projectedArea(i) / projectedArea(root)
		 */
		static void pah(float& totalPah, ANALYZER_ACTION_PER_NODE_ARGUMENTS) {
			//in order not to compute the projected area of the root each time, we keep the last projected area across calls...
			static const Bvh* lastBvh = &bvh;
			static float lastRootProjectedArea = bvh.getInfluenceArea().getProjectedArea(bvh.getRoot().aabb);
			//...and check whether we are calculating the PAH for a node of the same BVH as in the last call
			if (bvh != *lastBvh) {
				lastBvh = &bvh;
				lastRootProjectedArea = bvh.getInfluenceArea().getProjectedArea(bvh.getRoot().aabb);
			}

			float pah = Bvh::computeCostPah(node, bvh.getInfluenceArea(), lastRootProjectedArea);
			float pa = bvh.getInfluenceArea().getProjectedArea(node.aabb);

			//add to JSON
			localLog["metrics"]["pah"] = pah;
			localLog["metrics"]["pa"] = pa;

			//update global variable
			if (node.isLeaf()) { totalPah += pah; } //if a node is a leaf, its cost is actually the PAH: hitProbability * (costTriangle * #triangles)
			else { totalPah += (pa / lastRootProjectedArea) * 1.0f * 2.0f; } //if a node is internal, its cost is: hitProbability * (costNode * 2) where 2 is the number of children we now must visit
		}

		/**
		 * @brief Updates the maximum level of the BVH.
		 */
		static void levelCount(int& maxLevel, ANALYZER_ACTION_PER_NODE_ARGUMENTS) {
			maxLevel = currentLevel > maxLevel ? currentLevel : maxLevel;
		}
	}


	namespace finals {
		/**
		 * @brief Simply adds the number of nodes and leaves to the JSON.
		 */
		static void nodesAndLeaves(pair<int, int>& nodesAndLeaves, ANALYZER_ACTION_FINAL_ARGUMENTS) {
			log["globalInfo"]["numberOfNodes"] = nodesAndLeaves.first;
			log["globalInfo"]["numberOfLeaves"] = nodesAndLeaves.second;
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
	}
}
