#pragma once

#include "../libs/json.hpp"
#include "Bvh.h"
#include "Utilities.h"


namespace pah::analyzerLocalActions {
	static void coreAnalyzer(const Bvh::Node& node, int currentLevel, json& localLog) {
		localLog["core"] = node;
	}

	/**
	 * @brief Adds the section \c metric to the node Json object, and adds the node SAH and surface area in the new section.
	 */
	static void sahAnalyzer(const Bvh::Node& node, const Bvh& bvh, json& localLog) {
		localLog["metrics"]["sah"] = Bvh::computeCostSah(node, bvh.getInfluenceArea(), bvh.getRoot().aabb.surfaceArea());
		localLog["metrics"]["sa"] = node.aabb.surfaceArea();
	}

	/**
	 * @brief Adds the section \c metric to the node Json object, and adds the node PAH and projected area in the new section.
	 */
	static void pahAnalyzer(const Bvh::Node& node, const Bvh& bvh, json& localLog) {
		//in order not to compute the projected area of the root each time, we keep the last projected area across calls...
		static const Bvh* lastBvh = &bvh;
		static float lastRootProjectedArea = bvh.getInfluenceArea().getProjectedArea(bvh.getRoot().aabb);
		//...and check whether we are calculating the PAH for a node of the same BVH as in the last call
		if (bvh != *lastBvh) {
			lastBvh = &bvh;
			lastRootProjectedArea = bvh.getInfluenceArea().getProjectedArea(bvh.getRoot().aabb);
		}

		localLog["metrics"]["pah"] = Bvh::computeCostPah(node, bvh.getInfluenceArea(), lastRootProjectedArea);
		localLog["metrics"]["pa"] = lastRootProjectedArea;
	}
}


namespace pah::analyzerGlobalActions {
	namespace finalActions {

	}

	namespace perNodeActions {
		static void levelPerNode(int& levelInfo, const Bvh::Node& node, const Bvh& bvh, json& localLog) {

		}
	}
}
