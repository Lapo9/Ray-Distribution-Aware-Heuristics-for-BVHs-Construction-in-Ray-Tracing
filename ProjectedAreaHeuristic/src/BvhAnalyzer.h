#pragma once

#include <vector>
#include <functional>
#include <fstream>
#include "../libs/json.hpp"

#include "Utilities.h"
#include "Bvh.h"
#include "CustomJson.h"


namespace pah {

	/**
	 * @brief Class used to analyze a @p Bvh. The way in which the @p Bvh is analyzed is customizable.
	 */
	template<typename... GlobalObject>
	class BvhAnalyzer {
		using json = nlohmann::json;

	public:
		/**
		 * @brief Constructs a @p BvhAnalyzer where, at each node, the first action of the pair is executed, and at the end of the analysis, the second action is executed.
		 * The first argument of both actions must be a trivially initializable reference to an object. 
		 * Per-node actions can store data in this object, and the stored data can be accessed during the final action.
		 */
		BvhAnalyzer(std::pair<std::function<PerNodeActionType>, std::function<FinalActionType>>... actions) : actions{ std::make_tuple(actions...) } {}
		BvhAnalyzer() {}


		/**
		 * @brief Adds a pair of per-node function and final function to the list of global actions at position @p Pos.
		 *
		 * @tparam Pos the position where to insert these functions. It must be 0 <= Pos < size(GlobalObject...)
		 * @tparam T The type of the first arguments of the functions. It is required that it matches the type of the Pos-th element of @p globalObjects
		 * @param perNodeFunction Function to perform on each node.
		 * @param finalFunction Function to perform at the end.
		 */
		template<std::size_t Pos, std::same_as<std::tuple_element_t<Pos, std::tuple<GlobalObject...>>> T> requires (Pos >= 0 && Pos < tuple_size_v<std::tuple<GlobalObject...>>)
		void addAction(std::function<void(T&, const Bvh::Node&, const Bvh&, int currLvl, json&)> perNodeFunction, std::function<void(T&, json&)> finalFunction) {
			std::pair functions{ perNodeFunction, finalFunction };
			std::get<Pos>(actions) = functions;
		}

		/**
		 * @brief Does the same as @p addGlobalActionsIndex, but in this case it is not required to specify the position.
		 * The position it deduced by the type of the first argument in the functions.
		 * It is fundamental to note that, in case more than one function with the same first argument is present (a.k.a. more than one @p globalObject has the same type), thif function will fail at compile-time.
		 *
		 * @param perNodeFunction Function to perform on each node.
		 * @param finalFunction Function to perform at the end.
		 */
		template<typename T>
		void addAction(std::function<void(T&, const Bvh::Node&, const Bvh&, int currLvl, json&)> perNodeFunction, std::function<void(T&, json&)> finalFunction) {
			using namespace std;
			pair functions{ perNodeFunction, finalFunction };
			std::get<pair<function<void(T&, const Bvh::Node&, const Bvh&, int, json&)>, function<void(T&, json&)>>>(actions) = functions; //fails if there are more global objects with the same type (or none)
		}

		/**
		 * @brief Adds all the actions to the the @p BvhAnalyzer. Each action must match the type specified during the construction of the @p BvhAnalyzer.
		 */
		void addActions(std::pair<std::function<PerNodeActionType>, std::function<FinalActionType>>... actions) {
			this->actions = std::make_tuple(actions...);
		}

		/**
		 * @brief Given a @p Bvh, it analyzes it and returns a JSON.
		 */
		json analyze(const Bvh& bvh) {
			log = json{}; //initialize log string
			
			globalObjects = std::tuple<GlobalObject...>{}; //reset global objects
			analyzeNode(bvh.getRoot(), bvh, 0); //recurse on all nodes, depth-first

			//execute the final actions for each global object. For example, here it is possible to add to the json the info collected during the visit
			performFinalActions(bvh, log);

			return log;
		}

		/**
		 * @brief Given a @p Bvh, it analyzes it and returns a JSON. Moreover it saves the JSON to a file.
		 */
		json analyze(const Bvh& bvh, std::string filePath) {
			json json = analyze(bvh);

			std::ofstream file{ filePath };
			file << std::setw(2) << json;
			file.close();

			return json;
		}

	private:
		/**
		 * @brief Basically this does a for-each on all the elements of the @p actions and @p globalObjects tuple.
		 * What happens is that, for each function in @p actions, it calls it with the corresponding @p globalObject as argument (plus other possible arguments).
		 * The implementation is carried out in @p performGlobalActionsImpl, this wrapper lacks the boilerplate arguments.
		 * In this case we call the final functions.
		 */
		void performFinalActions(const Bvh& bvh, json& log) {
			constexpr auto Size = std::tuple_size_v<std::tuple<GlobalObject...>>; //the size of the tuple
			performFinalActionsImpl(std::make_index_sequence<Size>{}, bvh, log); //call with an index sequence from 0 to Size
		}

		/**
		 * @brief Basically this does a for-each on all the elements of the @p actions and \globalObjects tuple.
		 * What happens is that, for each function in @p actions, it calls it with the corresponding @p globalObject as argument (plus other possible arguments)
		 * In this case we call the final functions.
		 *
		 * @tparam Is A compile-time sequence of integer numbers. It is used to iterate over the tuple (it must be done at compile time).
		 * @param The index sequence. We don't care to store it, the important thing is that it can be used to deduce the template parameter.
		 */
		template<std::size_t... Is>
		void performFinalActionsImpl(std::index_sequence<Is...>, const Bvh& bvh, json& log) {
			//extract the I-th of the tuples globalObjects and actions, then perform the final function
			auto performGlobalAction = [&]<std::size_t I>() {
				const auto& action = std::get<I>(actions);
				auto& object = std::get<I>(globalObjects); //the & is fundamental, else we won't store a reference of the object in the tuple
				action.second(object, bvh, log);
			};

			(performGlobalAction.template operator() < Is > (), ...); //execute for each element in the tuple

			//this one-liner does the same, but I find it less clear
			"(std::get<Is>(actions).second(std::get<Is>(globalObjects), log), ...)";
		}

		/**
		 * @brief Basically this does a for-each on all the elements of the @p actions and \globalObjects tuple.
		 * What happens is that, for each function in @p actions, it calls it with the corresponding @p globalObject as argument (plus other possible arguments).
		 * The implementation is carried out in @p performGlobalActionsImpl, this wrapper lacks the boilerplate arguments.
		 * In this case we call the per-node functions.
		 *
		 * @tparam Size The size of the tuple. Is is defaulted to the size of a tuple of the same type as @p globalObjects .
		 */
		template<std::size_t Size = std::tuple_size_v<std::tuple<GlobalObject...>>>
		void performPerNodeActions(const Bvh::Node& node, const Bvh& bvh, int currentLevel, json& localLog) {
			performPerNodeActionsImpl(std::make_index_sequence<Size>{}, node, bvh, currentLevel, localLog); //call with an index sequence from 0 to Size
		}

		/**
		 * @brief Basically this does a for-each on all the elements of the @p actions and \globalObjects tuple.
		 * What happens is that, for each function in @p actions, it calls it with the corresponding @p globalObject as argument (plus other possible arguments)
		 * In this case we call the per-node functions.
		 *
		 * @tparam Is A compile-time sequence of integer numbers. It is used to iterate over the tuple (it must be done at compile time).
		 * @param The index sequence. We don't care to store it, the important thing is that it can be used to deduce the template parameter.
		 */
		template<std::size_t... Is>
		void performPerNodeActionsImpl(std::index_sequence<Is...>, const Bvh::Node& node, const Bvh& bvh, int currentLevel, json& localLog) {
			//extract the I-th of the tuples globalObjects and actions, then perform the per-node function
			auto performGlobalAction = [&]<std::size_t I>() {
				const auto& action = std::get<I>(actions);
				auto& object = std::get<I>(globalObjects); //the & is fundamental, else we won't store a reference of the object in the tuple
				action.first(object, node, bvh, currentLevel, localLog);
			};

			(performGlobalAction.template operator() < Is > (), ...); //execute for each element in the tuple

			//this one-liner does the same, but I find it less clear
			"(std::get<Is>(actions).first(std::get<Is>(globalObjects), node, bvh, localLog), ...)";
		}

		/**
		 * @brief Calls all the register actions on this specific node.
		 */
		void analyzeNode(const Bvh::Node& node, const Bvh& bvh, int currentLevel) {
			json localLog;

			//execute per-node actions with a global object associated (where they can save info across calls)
			performPerNodeActions(node, bvh, currentLevel, localLog);

			log["nodes"].push_back(localLog); //add log of this node to the global log

			//recursion
			currentLevel++;
			if (node.leftChild != nullptr) analyzeNode(*node.leftChild, bvh, currentLevel);
			if (node.rightChild != nullptr) analyzeNode(*node.rightChild, bvh, currentLevel);
		}


		std::tuple<GlobalObject...> globalObjects; //objects used by the global actions. Each global action has a corresponding object where it can store info across calls of analyzeNode
		std::tuple<std::pair<std::function<PerNodeActionType>, std::function<FinalActionType>>...> actions; //each pair of global actions has an associated object. The first function is responsible to update this object during the visit of each node. The second one is responsible to finalize the results by using the informations stored in the object.

		json log;
	};
}