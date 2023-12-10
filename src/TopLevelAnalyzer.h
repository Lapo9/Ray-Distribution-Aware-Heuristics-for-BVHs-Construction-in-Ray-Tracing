#pragma once

#include <tuple>
#include <functional>
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
			std::apply([&analyzer](auto&... go) { analyzer.addActions(go...); }, actions); //all all the actions to the analyzer

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
}
