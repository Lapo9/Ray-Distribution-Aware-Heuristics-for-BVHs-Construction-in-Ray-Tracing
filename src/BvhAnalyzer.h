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

		
		BvhAnalyzer(const vector<BvhAnalyzerActionType>& actions, pair<function<void(G&, const Bvh::Node&, const Bvh&, json&)>, function<void(G&)>>... globalActions) : actions{ actions }, globalInfo {} {
			this->globalActions = make_tuple(globalActions...);
		}

		BvhAnalyzer(pair<function<void(G&, const Bvh::Node&, const Bvh&, json&)>, function<void(G&)>>... globalActions) : actions{}, globalInfo{} {
			this->globalActions = make_tuple(globalActions...);
		}

		BvhAnalyzer(const vector<BvhAnalyzerActionType>& actions) : actions{ actions }, globalInfo{} {}

		BvhAnalyzer(){}


		/**
		 * @brief Adds a pair of per-node function and final function to the list of global actions at position \p Pos.
		 * 
		 * @tparam Pos the position where to insert these functions. It must be 0 <= Pos < size(G...)
		 * @tparam T The type of the first arguments of the functions. It is required that it matches the type of the Pos-th element of \p globalObjects
		 * @param perNodeFunction Function to perform on each node.
		 * @param finalFunction Function to perform at the end.
		 */
		template<std::size_t Pos, same_as<tuple_element_t<Pos, tuple<G...>>> T> requires (Pos >= 0 && Pos < tuple_size_v<tuple<G...>>)
		void addGlobalActionsIndex(function<void(T&, const Bvh::Node&, const Bvh&, json&)> perNodeFunction, function<void(T&)> finalFunction) {
			pair functions{ perNodeFunction, finalFunction };
			std::get<Pos>(globalActions) = functions;
		}

		/**
		 * @brief Does the same as \p addGlobalActionsIndex, but in this case it is not required to specify the position.
		 * The position it deduced by the type of the first argument in the functions.
		 * It is fundamental to note that, in case more than one function with the same first argument is present (a.k.a. more than one \c globalObject has the same type), thif function will fail at compile-time.
		 * 
		 * @param perNodeFunction Function to perform on each node.
		 * @param finalFunction Function to perform at the end.
		 */
		template<typename T>
			void addGlobalActions(function<void(T&, const Bvh::Node&, const Bvh&, json&)> perNodeFunction, function<void(T&)> finalFunction) {
			pair functions{ perNodeFunction, finalFunction };
			std::get<pair<function<void(T&, const Bvh::Node&, const Bvh&, json&)>, function<void(T&)>>>(globalActions) = functions; //fails if there are more global objects with the same type (or none)
		}

		/**
		 * @brief Adds the function to the list of per node actions to perform.
		 * 
		 * @param action This function is executed each time a node is visited.
		 */
		void addLocalAction(BvhAnalyzerActionType action) {
			actions.push_back(action);
		}

		/**
		 * @brief Given a BVH, it analyzes it and returns a Json.
		 */
		json analyze(const Bvh& bvh) {
			log = json{}; //initialize log string
			globalInfo = GlobalInfo{};

			analyzeNode(bvh.getRoot(), bvh, 0); //recurse on all nodes, depth-first

			//execute the final actions for each global object. For example, here it is possible to add to the json the info collected during the visit
			performFinalGlobalActions(); 

			return log;
		}


		/**
		 * @brief Basically this does a for-each on all the elements of the \p globalActions and \globalObjects tuple.
		 * What happens is that, for each function in \p globalActions, it calls it with the corresponding \p globalObject as argument (plus other possible arguments).
		 * The implementation is carried out in \p performGlobalActionsImpl, this wrapper lacks the boilerplate arguments.
		 * In this case we call the final functions.
		 *
		 * @tparam Size The size of the tuple. Is is defaulted to the size of a tuple of the same type as \p globalObjects .
		 */
		template<std::size_t Size = std::tuple_size_v<tuple<G...>>>
		void performFinalGlobalActions() {
			performFinalGlobalActionsImpl(std::make_index_sequence<Size>{}); //call with an index sequence from 0 to Size
		}

		/**
		 * @brief Basically this does a for-each on all the elements of the \p globalActions and \globalObjects tuple.
		 * What happens is that, for each function in \p globalActions, it calls it with the corresponding \p globalObject as argument (plus other possible arguments)
		 * In this case we call the final functions.
		 *
		 * @tparam Is A compile-time sequence of integer numbers. It is used to iterate over the tuple (it must be done at compile time).
		 * @param The index sequence. We don't care to store it, the important thing is that it can be used to deduce the template parameter.
		 */
		template<std::size_t... Is>
		void performFinalGlobalActionsImpl(std::index_sequence<Is...>) {
			//extract the I-th of the tuples globalObjects and globalActions, then perform the final function
			auto performGlobalAction = [&]<std::size_t I>() {
				const auto& action = std::get<I>(globalActions);
				auto& object = std::get<I>(globalObjects); //the & is fundamental, else we won't store a reference of the object in the tuple
				action.second(object);
			};
			
			(performGlobalAction.template operator() <Is> (), ...); //execute for each element in the tuple

			//this one-liner does the same, but I find it less clear
			"(std::get<Is>(globalActions).second(std::get<Is>(globalObjects)), ...)";
		}		

		/**
		 * @brief Basically this does a for-each on all the elements of the \p globalActions and \globalObjects tuple.
		 * What happens is that, for each function in \p globalActions, it calls it with the corresponding \p globalObject as argument (plus other possible arguments).
		 * The implementation is carried out in \p performGlobalActionsImpl, this wrapper lacks the boilerplate arguments.
		 * In this case we call the per-node functions.
		 *
		 * @tparam Size The size of the tuple. Is is defaulted to the size of a tuple of the same type as \p globalObjects .
		 */
		template<std::size_t Size = std::tuple_size_v<tuple<G...>>>
		void performPerNodeGlobalActions(const Bvh::Node& node, const Bvh& bvh, json& localLog) {
			performPerNodeGlobalActionsImpl(std::make_index_sequence<Size>{}, node, bvh, localLog); //call with an index sequence from 0 to Size
		}

		/**
		 * @brief Basically this does a for-each on all the elements of the \p globalActions and \globalObjects tuple.
		 * What happens is that, for each function in \p globalActions, it calls it with the corresponding \p globalObject as argument (plus other possible arguments)
		 * In this case we call the per-node functions.
		 * 
		 * @tparam Is A compile-time sequence of integer numbers. It is used to iterate over the tuple (it must be done at compile time).
		 * @param The index sequence. We don't care to store it, the important thing is that it can be used to deduce the template parameter.
		 */
		template<std::size_t... Is>
		void performPerNodeGlobalActionsImpl(std::index_sequence<Is...>, const Bvh::Node& node, const Bvh& bvh, json& localLog) {
			//extract the I-th of the tuples globalObjects and globalActions, then perform the per-node function
			auto performGlobalAction = [&]<std::size_t I>() {
				const auto& action = std::get<I>(globalActions);
				auto& object = std::get<I>(globalObjects); //the & is fundamental, else we won't store a reference of the object in the tuple
				action.first(object, node, bvh, localLog);
			};

			(performGlobalAction.template operator() < Is > (), ...); //execute for each element in the tuple

			//this one-liner does the same, but I find it less clear
			"(std::get<Is>(globalActions).first(std::get<Is>(globalObjects), node, bvh, localLog), ...)";
		}
		


	private:
		void analyzeNode(const Bvh::Node& node, const Bvh& bvh, int currentLevel) {
			json localLog;
			coreAction(node, currentLevel, localLog);
			//execute per-node actions
			for (const auto& action : actions) {
				action(node, bvh, localLog);
			}
			performPerNodeGlobalActions(node, bvh, localLog); //execute per-node actions with a global object associated (where they can save info across calls)

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
		tuple<G...> globalObjects; //objects used by the global actions. Each global action has a corresponding object where it can store info across calls of analyzeNode
		tuple<pair<function<void(G&, const Bvh::Node&, const Bvh&, json&)>..., function<void(G&)>...>> globalActions; //each pair of global actions has an associated object. The first function is responsible to update this object during the visit of each node. The second one is responsible to finalize the results by using the informations stored in the object.

		GlobalInfo globalInfo;
		json log;
	};


	/**
	 * @brief Adds the section \c metric to the node Json object, and adds the node SAH and surface area in the new section.
	 */
	void sahAnalyzer(const Bvh::Node& node, const Bvh& bvh, json& localLog);

	/**
	 * @brief Adds the section \c metric to the node Json object, and adds the node PAH and projected area in the new section.
	 */
	void pahAnalyzer(const Bvh::Node& node, const Bvh& bvh, json& localLog);
}
