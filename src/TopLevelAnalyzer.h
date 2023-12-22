#pragma once

#include <tuple>
#include <functional>
#include <queue>
#include "../libs/json.hpp"

#include "Utilities.h"
#include "Bvh.h"
#include "BvhAnalyzer.h"
#include "TopLevel.h"

namespace pah {

	template<typename... GlobalObject>
	class TopLevelAnalyzer {
		using json = nlohmann::json;

	public:
		TopLevelAnalyzer(std::pair<std::function<PerNodeActionType>, std::function<FinalActionType>>... actions) : actions{ std::make_tuple(actions...) } {}

		/**
		 * @brief Given a top level structure, it analyzes it and returns a JSON.
		 */
		json analyze(const TopLevel& topLevel) {
			BvhAnalyzer<GlobalObject...> analyzer{};
			std::apply([&analyzer](auto&... go) { analyzer.addActions(go...); }, actions); //all the actions to the analyzer

			json analyses;
			for (auto& bvh : topLevel.getBvhs()) {
				analyses["bvhs"] += analyzer.analyze(bvh); //perform the analyses of every BVH
			}

			for (auto& t : topLevel.getTriangles()) {
				analyses["triangles"] += t;
			}

			return analyses;
		}

		/**
		 * @brief Given a top level structure, it analyzes it and returns a JSON. Moreover it saves the JSON to a file.
		 */
		json analyze(const TopLevel& topLevel, std::string filePath) {
			json json = analyze(topLevel);

			std::ofstream file;
			file.open(filePath);
			file << std::setw(2) << json;
			file.close();

			return json;
		}

	private:
		std::tuple<std::pair<std::function<PerNodeActionType>, std::function<FinalActionType>>...> actions; //actions we'll pass to each BVH analyzer
	};


	template<typename... GlobalObject>
	class TopLevelOctreeAnalyzer {
		using json = nlohmann::json;

	public:
		TopLevelOctreeAnalyzer(std::pair<std::function<PerNodeActionType>, std::function<FinalActionType>>... actions) : topLevelAnalyzer{ actions... } {}

		json analyze(const TopLevelOctree& topLevel) {
			json analyses = topLevelAnalyzer.analyze(topLevel); //get the analyses of all the BVHs

			//now analyze the octree
			std::queue<TopLevelOctree::Node*> toAnalyze;
			toAnalyze.push(&topLevel.getRoot());
			while (!toAnalyze.empty()) {
				auto& currentNode = toAnalyze.front();
				toAnalyze.pop(); //remove the element we got via front()
				analyses["octree"] += *currentNode;
				if (currentNode->isLeaf()) continue;
				for(auto& childPtr : currentNode->children) toAnalyze.push(&*childPtr); //add all children to the queue				
			}	

			return analyses;
		}

		json analyze(const TopLevelOctree& topLevel, std::string filePath) {
			json json = analyze(topLevel);

			std::ofstream file;
			file.open(filePath);
			file << std::setw(2) << json;
			file.close();

			return json;
		}

	private:
		TopLevelAnalyzer<GlobalObject...> topLevelAnalyzer;
	};
}
