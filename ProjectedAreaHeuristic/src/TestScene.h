#pragma once

#include <vector>
#include <ranges>
#include "Utilities.h"
#include "Bvh.h"
#include "TopLevel.h"
#include "RayCaster.h"
#include "CustomJson.h"
#include "TopLevelAnalyzer.h"

namespace pah {
	
	/**
	 * @brief A TestScene holds a TopLevel acceleration structure and a collection of RayCasters. It also holds a reference to a collection of triangles.
	 * It can be then used to build the TopLevel and cast rays on those, collecting the traversal results.
	 */
	template<typename VectorOfRayCasters, typename TopLevelAnalyzer>//, typename... GlobalObject>
	class TestScene {
	public:

		template<typename VectorOfRayCastersType, typename TopLevelAnalyzerType, typename TopLevelType>
		TestScene(const std::string& outputFolderPath, const std::vector<Triangle>& triangles, VectorOfRayCastersType&& rayCasters, TopLevelType&& topLevel, TopLevelAnalyzerType&& topLevelAnalyzer) :
			triangles{ triangles }, rayCasters{ std::forward<VectorOfRayCasters>(rayCasters) }, outputFolderPath{ outputFolderPath } {
			this->topLevel = std::make_unique<TopLevel>(std::forward<TopLevel>(topLevel));
			this->topLevelAnalyzer = std::make_unique<TopLevelAnalyzer>(std::forward<TopLevelAnalyzer>(topLevelAnalyzer));
		}

		void build() {
			topLevel->build(triangles);
		}

		std::tuple<json, CumulativeRayCasterResults, CumulativeRayCasterResults> traverse() {
			json constructionAnalysis;// = topLevelAnalyzer->analyze(topLevel, outputFolderPath + "/ConstructionAnalysis.json");

			auto topLevelResults = std::ranges::fold_left(rayCasters, CumulativeRayCasterResults{}, [this](auto res, auto rayCaster) { return res + rayCaster->castRays(*topLevel); });
			std::ofstream topLevelResultsFile{ outputFolderPath + "/TopLevelTraversalResults.json" };
			topLevelResultsFile << std::setw(2) << json(topLevelResults);
			topLevelResultsFile.close();

			auto fallbackSahResults = std::ranges::fold_left(rayCasters, CumulativeRayCasterResults{}, [this](auto res, auto rayCaster) { return res + rayCaster->castRays(topLevel->getFallbackBvh()); });
			std::ofstream fallbackSahResultsFile{ outputFolderPath + "/FallbackSahTraversalResults.json" };
			fallbackSahResultsFile << std::setw(2) << json(topLevelResults);
			fallbackSahResultsFile.close();

			return std::tuple{ constructionAnalysis , topLevelResults, fallbackSahResults };
		}

		std::tuple<json, CumulativeRayCasterResults, CumulativeRayCasterResults> buildAndTraverse() {
			build();
			return traverse();
		}

	private:
		const std::vector<Triangle> triangles;
		VectorOfRayCasters rayCasters;
		std::unique_ptr<TopLevel> topLevel; // it must be a pointer for polymorphism
		std::unique_ptr<TopLevelAnalyzer> topLevelAnalyzer;
		std::string outputFolderPath;
	};


	template<typename VectorOfRayCastersType, typename TopLevelAnalyzerType, typename TopLevelType>
	TestScene(const std::string&, const std::vector<Triangle>&, VectorOfRayCastersType&&, TopLevelType&&, TopLevelAnalyzerType&&)
		-> TestScene<std::remove_reference_t<VectorOfRayCastersType>, std::remove_reference_t<TopLevelAnalyzerType>>;
}

