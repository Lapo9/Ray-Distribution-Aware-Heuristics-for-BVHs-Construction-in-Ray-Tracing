#pragma once

#include <vector>
#include <functional>
#include "../libs/json.hpp"

#include "Utilities.h"
#include "Bvh.h"
#include "CustomJson.h"


namespace pah {
	using json = nlohmann::json;

	template<typename... G>
	class BvhAnalyzer {
	public:
		using BvhAnalyzerActionType = void(*)(const Bvh::Node& node, const Bvh& bvh, json& localLog);

		struct GlobalInfo {
			int numberOfNodes;
			int numberOfLeaves;
			int maxLevel;
			

			GlobalInfo() : numberOfNodes{ 0 }, numberOfLeaves{ 0 }, maxLevel{ 0 } {}
		};

		
		BvhAnalyzer(const vector<BvhAnalyzerActionType>& actions, pair<function<void(G&, float)>, function<void(G&)>>... globalActions) : actions{ actions }, globalInfo {} {
			this->globalActions = make_tuple(globalActions...);
		}

		/**
		 * @brief Given a BVH, it analyzes it and returns a Json.
		 */
		json analyze(const Bvh& bvh) {
			log = json{}; //initialize log string
			globalInfo = GlobalInfo{};

			analyzeNode(bvh.getRoot(), bvh, 0);

			return log;
		}


		template<std::size_t... I>
		void performGlobalActions() {
			//auto action = std::get<I>(globalActions);
			//auto object = std::get<I>(globalObjects);
			(std::get<I>(globalActions).first(std::get<I>(globalObjects), 0.1f), ...);
		}
	private:
		void analyzeNode(const Bvh::Node& node, const Bvh& bvh, int currentLevel) {
			json localLog;
			coreAction(node, currentLevel, localLog);
			for (const auto& action : actions) {
				action(node, bvh, localLog);
			}

			log["nodes"].push_back(localLog); //add log of this node to the global log

			//recursion
			if (node.leftChild != nullptr) analyzeNode(*node.leftChild, bvh, currentLevel);
			if (node.rightChild != nullptr) analyzeNode(*node.rightChild, bvh, currentLevel);
		}

		void coreAction(const Bvh::Node& node, int currentLevel, json& localLog) {
			globalInfo.maxLevel = currentLevel > globalInfo.maxLevel ? currentLevel : globalInfo.maxLevel;
			globalInfo.numberOfNodes++;
			globalInfo.numberOfLeaves += node.isLeaf();

			localLog["core"] = node;
		}

		vector<BvhAnalyzerActionType> actions; //functions called during the visit of each node
		tuple<G...> globalObjects; //objects used by the global actions. Each global action has a corresponding object
		tuple<pair<function<void(G&, float)>..., function<void(G&)>...>> globalActions; //each pair of global actions has an associated object. The first function is responsible to update this object during the visit of each node. The second one is responsible to finalize the results by using the informations stored in the object.

		GlobalInfo globalInfo;
		json log;
	};


	/**
	 * @brief Adds the section \c metric to the node Json object, and adds the node SAH and surface area in the new section.
	 */
	static void sahAnalyzer(const Bvh::Node& node, const Bvh& bvh, json& localLog);

	/**
	 * @brief Adds the section \c metric to the node Json object, and adds the node PAH and projected area in the new section.
	 */
	static void pahAnalyzer(const Bvh::Node& node, const Bvh& bvh, json& localLog);
}
