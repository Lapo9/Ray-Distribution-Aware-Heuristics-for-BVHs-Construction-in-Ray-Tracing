#pragma once

#include <unordered_map>
#include <vector>
#include <functional>
#include <string>
#include <tuple>

#include "CustomJson.h"


namespace pah {

	/**
	 * @brief Via objects of this class it is possible to export some specific data from a @p TestScene analysis in csv format.
	 */
	class CsvExporter {
	public:

		/**
		 * @brief Builds an object with the specified accessors.
		 * An accessor is a pair containing a name for the column in the csv file, and a lambda that, given a @p TestScene analysis, returns a specific value of that analysis.
		 * e.g. it is possible to access the PAH cost of a TopLevel structure, or the traversal time of a Bvh.
		 */
		template<std::convertible_to<std::function<std::string(const std::tuple<json, json, json>&)>>... Accessors>
		CsvExporter(std::pair<std::string, Accessors>... accessors) {
			addAccessors(accessors...);
		}

		/**
		 * @brief Adds a new @TestScene analysis from which to extract values (i.e. the rows of the csv file).
		 * Each analysis must have a unique name.
		 */
		void addAnalysis(std::string name, std::tuple<json, json, json>&& analysis) {
			analyses.emplace(name, std::move(analysis));
		}

		/**
		 * @brief Adds a new accessor (i.e. the columns of the csv file).
		 * 
		 * @param ...accessors .
		 */
		template<std::convertible_to<std::function<std::string(const std::tuple<json, json, json>&)>>... Accessors>
		void addAccessors(std::pair<std::string, Accessors>... accessors) {
			(this->accessors.push_back(accessors), ...);
		}

		/**
		 * @brief Builds the csv file based on the analysis and accessors.
		 * 
		 * @param path .
		 */
		void generateCsv(std::string path) const {
			std::string csv;

			// add the column names
			csv += "Scene";
			for (const auto& [displayName, accessor] : accessors) {
				csv += "," + displayName;
			}
			csv += "\n";

			// add the data
			for (const auto& [scene, analysis] : analyses) {
				csv += scene;
				for (const auto& [displayName, accessor] : accessors) {
					csv += "," + accessor(analysis);
				}
				csv += "\n";
			}

			// print to file
			std::ofstream outputFile{ path };
			outputFile << csv;
			outputFile.close();
		}


	private:
		std::unordered_map<std::string, std::tuple<json, json, json>> analyses;
		std::vector<std::pair<std::string, std::function<std::string(const std::tuple<json, json, json>&)>>> accessors;
	};
}


enum class AnalysisType {
	TOP_LEVEL, PAH, FALLBACK
};

/**
 * @brief Macro used to create a pair <column name - lambda function to extract a value from a @TestScene analysis>.
 */
#define ACCESSOR(accessorName, analysisName, accessor)  std::pair{ \
															std::string(accessorName), \
															[](const std::tuple<json, json, json>& analyses) { \
																auto& [topLevel, pah, fallback] = analyses; \
																std::stringstream value; \
																switch(analysisName) { \
																	case AnalysisType::TOP_LEVEL: \
																		value << topLevel accessor; \
																		break; \
																	case AnalysisType::PAH: \
																		value << pah accessor; \
																		break; \
																	case AnalysisType::FALLBACK: \
																		value << fallback accessor; \
																		break; \
																} \
																return value.str(); \
															} \
														}
