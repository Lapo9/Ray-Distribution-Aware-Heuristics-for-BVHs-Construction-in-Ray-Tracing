#pragma once

#include <vector>
#include <functional>
#include "../libs/json.hpp"

#include "Utilities.h"
#include "Bvh.h"


namespace pah {
	using json = nlohmann::json;

	class BvhAnalyzer {
	public:
		using BvhAnalyzerActionType = void(const Bvh::Node& node, const Bvh& bvh, json& localLog);

		struct GlobalInfo {
			int numberOfNodes;
			int numberOfLeaves;
			int maxLevel;

			GlobalInfo() : numberOfNodes{ 0 }, numberOfLeaves{ 0 }, maxLevel{ 0 } {}
		};

		template<same_as<BvhAnalyzerActionType>... A> 
		BvhAnalyzer(const A&... actions) : globalInfo{} {
			(this->actions.push_back(actions), ...);
		}

		/**
		 * @brief Given a BVH, it analyzes it and returns a Json.
		 */
		json analyze(const Bvh& bvh);


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


	private:
		void analyzeNode(const Bvh::Node& node, const Bvh& bvh, int currentLevel);

		void coreAction(const Bvh::Node& node, int currentLevel, json& localLog);

		vector<function<BvhAnalyzerActionType>> actions;
		GlobalInfo globalInfo;
		json log;
	};
}
