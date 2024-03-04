#pragma once

#include <vector>
#include <ranges>
#include <filesystem>
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
	template<typename VectorOfRayCasters, typename TopLevelAnalyzer>
	class TestScene {
	public:

		/**
		 * @brief This is a template because we want to use universal references in order to be able to "steal" an object or copy it.
		 * The issue is that a universal reference must be a plain template type (so no template template parameters), and due to a bug in MSVC they cannot even be constrained with concepts or requires clauses.
		 */
		template<typename VectorOfRayCastersType, typename TopLevelAnalyzerType, typename TopLevelType>
		TestScene(const std::string& outputFolderPath, const std::vector<Triangle>& triangles, VectorOfRayCastersType&& rayCasters, TopLevelType&& topLevel, TopLevelAnalyzerType&& topLevelAnalyzer) :
			triangles{ triangles }, rayCasters{ std::forward<VectorOfRayCastersType>(rayCasters) }, outputFolderPath{ outputFolderPath } {
			this->topLevel = std::make_unique<std::remove_reference_t<TopLevelType>>(std::forward<TopLevelType>(topLevel));
			this->topLevelAnalyzer = std::make_unique<TopLevelAnalyzer>(std::forward<TopLevelAnalyzerType>(topLevelAnalyzer));

			std::filesystem::create_directory(outputFolderPath);
		}

		/**
		 * @brief Builds the TopLevel structure and analyzes it (if required). Returns the analysis.
		 * 
		 */
		json build(bool analyze = true) {
			topLevel->build(triangles);
			if(analyze) return topLevelAnalyzer->analyze(*topLevel, outputFolderPath + "/ConstructionAnalysis.json");
			return json{};
		}

		/**
		 * @brief Traverses the TopLevel structure and the fallback Bvh alone, and returns the traversal results.
		 */
		std::pair<CumulativeRayCasterResults, CumulativeRayCasterResults> traverse() {
			auto topLevelResults = std::ranges::fold_left(rayCasters, CumulativeRayCasterResults{}, [this](auto res, auto rayCaster) { return res + rayCaster->castRays(*topLevel); });
			std::ofstream topLevelResultsFile{ outputFolderPath + "/TopLevelTraversalResults.json" };
			topLevelResultsFile << std::setw(2) << json(topLevelResults);
			topLevelResultsFile.close();

			auto fallbackSahResults = std::ranges::fold_left(rayCasters, CumulativeRayCasterResults{}, [this](auto res, auto rayCaster) { return res + rayCaster->castRays(topLevel->getFallbackBvh()); });
			std::ofstream fallbackSahResultsFile{ outputFolderPath + "/FallbackSahTraversalResults.json" };
			fallbackSahResultsFile << std::setw(2) << json(fallbackSahResults);
			fallbackSahResultsFile.close();

			return { topLevelResults, fallbackSahResults };
		}

		/**
		 * @brief Basizally calls build and then analyze, and returns the combined results.
		 */
		std::tuple<json, CumulativeRayCasterResults, CumulativeRayCasterResults> buildAndTraverse(bool analyzeTopLevel = true) {
			json analysis = build(analyzeTopLevel);
			auto traversal = traverse();
			return { analysis, traversal.first, traversal.second };
		}

	private:
		const std::vector<Triangle> triangles;
		VectorOfRayCasters rayCasters;
		std::unique_ptr<TopLevel> topLevel; // it must be a pointer for polymorphism
		std::unique_ptr<TopLevelAnalyzer> topLevelAnalyzer; // it must be a pointer for polymorphism
		std::string outputFolderPath;
	};


	/**
	 * @brief Deduction guide for this class. 
	 * We must use a deduction guide instead of directly using the class template parameters in the ctor, because, since we want to get universal references, we cannot use class template parameters for them (but only function template parameters).
	 */
	template<typename VectorOfRayCastersType, typename TopLevelAnalyzerType, typename TopLevelType>
	TestScene(const std::string&, const std::vector<Triangle>&, VectorOfRayCastersType&&, TopLevelType&&, TopLevelAnalyzerType&&)
		-> TestScene<std::remove_reference_t<VectorOfRayCastersType>, std::remove_reference_t<TopLevelAnalyzerType>>;
}

