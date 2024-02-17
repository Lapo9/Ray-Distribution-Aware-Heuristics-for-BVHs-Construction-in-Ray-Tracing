#pragma once

#include <glm/glm.hpp>
#include <vector>
#include <memory>
#include <functional>
#include <array>
#include <algorithm>
#include <random>

#include "Utilities.h"
#include "InfluenceArea.h"
#include "settings.h"

namespace pah {
	/**
	 * @brief A BVH is a bounding volume hierarchy.
	 */
	class Bvh {
	public:
		//related classes
		/**
		 * @brief Contains info about the time it took to build a @p Bvh::Node.
		 */
		struct NodeTimingInfo {
			DurationMs total;
			DurationMs splittingTot;
			DurationMs computeCostTot;
			int computeCostCount;
			DurationMs splitTrianglesTot;
			int splitTrianglesCount;
			DurationMs chooseSplittingPlanesTot;
			int chooseSplittingPlanesCount;
			DurationMs shouldStopTot;
			int shouldStopCount;
			DurationMs nodesCreationTot;
			int nodesCreationCount;

			DurationMs computeCostMean() const { return computeCostTot / computeCostCount; }
			DurationMs splitTrianglesMean() const { return splitTrianglesTot / splitTrianglesCount; }
			DurationMs chooseSplittingPlanesMean() const { return chooseSplittingPlanesTot / chooseSplittingPlanesCount; }
			DurationMs shouldStopMean() const { return shouldStopTot / shouldStopCount; }
			DurationMs nodesCreationMean() const { return nodesCreationTot / nodesCreationCount; }
			
			void logTotal(DurationMs duration) {
				total = duration;
			}

			void logSplittingTot(DurationMs duration) {
				splittingTot = duration;
			}

			void logComputeCost(DurationMs duration) {
				computeCostTot += duration;
				computeCostCount++;
			}

			void logSplitTriangles(DurationMs duration) {
				splitTrianglesTot += duration;
				splitTrianglesCount++;
			}

			void logChooseSplittingPlanes(DurationMs duration) {
				chooseSplittingPlanesTot += duration;
				chooseSplittingPlanesCount++;
			}

			void logShouldStop(DurationMs duration) {
				shouldStopTot += duration;
				shouldStopCount++;
			}

			void logNodesCreation(DurationMs duration) {
				nodesCreationTot += duration;
				nodesCreationCount++;
			}

			/**
			 * @brief Adds together 2 NodeTimingInfo objects.
			 */
			NodeTimingInfo& operator+=(const NodeTimingInfo& lhs) {
				total = total + lhs.total;
				splittingTot = splittingTot + lhs.splittingTot;
				computeCostTot = computeCostTot + lhs.computeCostTot;
				computeCostCount = computeCostCount + lhs.computeCostCount;
				splitTrianglesTot = splitTrianglesTot + lhs.splitTrianglesTot;
				splitTrianglesCount = splitTrianglesCount + lhs.splitTrianglesCount;
				chooseSplittingPlanesTot = chooseSplittingPlanesTot + lhs.chooseSplittingPlanesTot;
				chooseSplittingPlanesCount = chooseSplittingPlanesCount + lhs.chooseSplittingPlanesCount;
				shouldStopTot = shouldStopTot + lhs.shouldStopTot;
				shouldStopCount = shouldStopCount + lhs.shouldStopCount;				
				nodesCreationTot = nodesCreationTot + lhs.nodesCreationTot;
				nodesCreationCount = nodesCreationCount + lhs.nodesCreationCount;

				return *this;
			}

			friend NodeTimingInfo& operator+(NodeTimingInfo lhs, const NodeTimingInfo& rhs) {
				return lhs += rhs;
			}

			friend bool operator==(const NodeTimingInfo& lhs, const NodeTimingInfo& rhs) {
				return	lhs.total == rhs.total &&
					lhs.splittingTot == rhs.splittingTot &&
					lhs.computeCostTot == rhs.computeCostTot &&
					lhs.computeCostCount == rhs.computeCostCount &&
					lhs.splitTrianglesTot == rhs.splitTrianglesTot &&
					lhs.splitTrianglesCount == rhs.splitTrianglesCount &&
					lhs.chooseSplittingPlanesTot == rhs.chooseSplittingPlanesTot &&
					lhs.chooseSplittingPlanesCount == lhs.chooseSplittingPlanesCount &&
					lhs.shouldStopTot == rhs.shouldStopTot &&
					lhs.shouldStopCount == rhs.shouldStopCount &&
					lhs.nodesCreationTot == rhs.nodesCreationTot &&
					lhs.nodesCreationCount == rhs.nodesCreationCount;
			}
		};

		/**
		 * @brief Node of a @p Bvh.
		 */
		struct Node {
			Aabb aabb;
			std::unique_ptr<Node> leftChild;
			std::unique_ptr<Node> rightChild;
			std::vector<const Triangle*> triangles;
			TIME(mutable NodeTimingInfo nodeTimingInfo;)

			/**
			 * @brief Creates an empty node with the specified @p Aabb.
			 */
			Node(Aabb aabb) : aabb{ aabb } {
				TIME(nodeTimingInfo = NodeTimingInfo{};);
			};
			/**
			 * @brief Creates a @p Node given a list of triangles. The @p Aabb of the node is the tightest one to enclose all the vertices of the triangles.
			 */
			Node(const std::vector<const Triangle*>& triangles) : aabb{ Aabb{triangles} }, triangles{ triangles } {
				TIME(nodeTimingInfo = NodeTimingInfo{};);
			}
			/**
			 * @brief Creates an empty @p Node.
			 */
			Node() : aabb{} {
				TIME(nodeTimingInfo = NodeTimingInfo{};);
			};

			/**
			 * @brief A @p Node is a leaf if both children are empty.
			 */
			bool isLeaf() const {
				return leftChild == nullptr && rightChild == nullptr;
			}
		};

		/**
		 * @brief Info about the results of a traversal of the @p Bvh of a @p Ray.
		 */
		struct TraversalResults {
			int intersectionTestsTotal;
			int intersectionTestsWithNodes;
			int intersectionTestsWithTriangles;
			float traversalCost;
			const Triangle* closestHit;
			float closestHitDistance;
			TIME(DurationMs traversalTime;);

			bool hit() const {
				return closestHit != nullptr;
			}
		};

		/**
		 * @brief Properties used to build the @p Bvh.
		 */
		struct Properties {
			float maxLeafCost; /**< If a @p Node cost is less than this threshold, it is automatically a leaf. */
			int maxTrianglesPerLeaf; /**< If a @p Node has less triangles than this threshold, it is automatically a leaf. */
			int maxLevels; /**< Max number of levels of the @p Bvh. */
			int bins; /**< How many bins are used to create the nodes of th @p Bvh. The higher, the better the @p Bvh, but the slower the construction. */

			Properties() = default;
			Properties(const Properties&) = default;
			Properties& operator=(const Properties&) = default;
		};

		//custom alias
		using ComputeCostReturnType = float;																				using ComputeCostType = ComputeCostReturnType(const Node&, const InfluenceArea&, float rootMetric);
		using ChooseSplittingPlanesReturnType = std::vector<std::tuple<Axis, std::function<bool(float bestCostSoFar)>>>;	using ChooseSplittingPlanesType = ChooseSplittingPlanesReturnType(const Node&, const InfluenceArea&, Axis father, std::mt19937& rng);
		using ShouldStopReturnType = bool;																					using ShouldStopType = ShouldStopReturnType(const Node&, const Properties&, int currentLevel, float nodeCost);


		Bvh(const Properties&, const InfluenceArea&, ComputeCostType computeCost, ChooseSplittingPlanesType chooseSplittingPlanes, ShouldStopType shouldStop);

		Bvh(const Properties&, ComputeCostType computeCost, ChooseSplittingPlanesType chooseSplittingPlanes, ShouldStopType shouldStop);

		/**
		 * @brief Returns whether @p bvh1 is the same as @p bvh2.
		 * To do so, it compares the ids of the 2 @p Bvh s. This is just a cheap way to understand if we are dealing with the exact same object.
		 * The id changes every time the @p Bvh is built.
		 */
		friend bool operator==(const Bvh& bvh1, const Bvh& bvh2) {
			return bvh1.id == bvh2.id;
		}

		/**
		 * @brief Constructs the @p Bvh on a set of triangles. The seed for the random operations during the construction is random.
		 */
		void build(const std::vector<const Triangle*>& triangles);
		/**
		 * @brief Constructs the @p Bvh on a set of triangles. It is possible to specify the seed for the random operations during the construction.
		 */
		void build(const std::vector<const Triangle*>& triangles, unsigned int seed);

		/**
		 * @brief Traverses the @p Bvh and returns some stats about the traversal.
		 */
		TraversalResults traverse(const Ray& ray) const;


		const Node& getRoot() const; /**< @brief Returns the root of the @p Bvh. */
		const InfluenceArea* getInfluenceArea() const; /**< @brief Returns the @p InfluenceArea of the @p Bvh. */
		INFO(const DurationMs getTotalBuildTime() const;); /**< @brief Returns the time it took to build this @p Bvh. */
		const Properties getProperties() const; /**< @brief Returns the properties of this @p Bvh. */

	private:
		/**
		 * @brief Given a @p Node, it splits it into 2 children according to the strategies set during @p Bvh construction.
		 */
		void splitNode(Node& node, Axis fathersplittingAxis, int currentLevel);

		//simple wrappers for the custom functions. We use wrappers because there may be some common actions to perform before (e.g. time logging)
		ComputeCostReturnType computeCostWrapper(const Node& parent, const Node& node, const InfluenceArea& influenceArea, float rootArea);
		ChooseSplittingPlanesReturnType chooseSplittingPlanesWrapper(const Node& node, const InfluenceArea& influenceArea, Axis axis, std::mt19937& rng);
		ShouldStopReturnType shouldStopWrapper(const Node& parent, const Node& node, const Properties& properties, int currentLevel, float nodeCost);

		/**
		 * @brief Given a list of triangles, an axis and a position on this axis, returns 2 sets of triangles, the ones "to the left" of the plane, and the ones "to the right".
		 */
		static std::tuple<std::vector<const Triangle*>, std::vector<const Triangle*>> splitTriangles(const Node& node, const std::vector<const Triangle*>& triangles, Axis axis, float splittingPlanePosition) {
			using namespace utilities;
			//the final action simply adds the measured time to the total split triangles time, and increases the split triangles counter
			TIME(TimeLogger timeLogger{ [&timingInfo = node.nodeTimingInfo](DurationMs duration) { timingInfo.splitTrianglesTot += duration; timingInfo.splitTrianglesCount++; } };);

			std::vector<const Triangle*> left, right;

			for (auto t : triangles) {
				if (at(t->barycenter(), axis) < splittingPlanePosition) left.push_back(t);
				else right.push_back(t);
			}

			return { left, right };
		}


		Node root;
		float rootMetric; //stores the cost metric of the root (e.g. surface area if we use SAH, projected area if we use PAH, ...)
		Properties properties;
		const InfluenceArea* influenceArea;

		//customizable functions
		std::function<ComputeCostType> computeCost;
		//the idea is that this function returns a list of suitable axis to try to subdivide the AABB into, and a set of corresponding predicates.
		//these predicates take into account the "quality" of the axis, and the results obtained from previous axis; and they decide whether is it worth it to try the next axis
		std::function<ChooseSplittingPlanesType> chooseSplittingPlanes;
		std::function<ShouldStopType> shouldStop;

		std::mt19937 rng; //random number generator
		INFO(DurationMs totalBuildTime;); //total time of the last build
		unsigned long long int id; //id of this BVH: it is a cheap way to check if 2 BVHs are equal
	};


	/**
	 * @brief Functions that can be plugged in the @p Bvh to decide how to build it.
	 */
	namespace bvhStrategies {

		/**
		 * @brief Computes the surface area heuristic of the specified node of a @p Bvh whose root has surface area @p rootSurfaceArea.
		 */
		static Bvh::ComputeCostReturnType computeCostSah(const Bvh::Node& node, const InfluenceArea&, float rootArea) {
			if (rootArea < 0) return node.aabb.surfaceArea(); //this function is called with rootArea < 0 when we want to initialize it

			float hitProbability = node.aabb.surfaceArea() / rootArea;
			float cost = node.isLeaf() ? 1.2f : 1.0f;
			return hitProbability * node.triangles.size() * cost;
		}

		/**
		 * @brief Computes the projected area heuristic of the specified node of a @p Bvh whose root has surface area @p rootSurfaceArea.
		 */
		static Bvh::ComputeCostReturnType computeCostPah(const Bvh::Node& node, const InfluenceArea& influenceArea, float rootProjectedArea) {
			if (rootProjectedArea < 0) return influenceArea.getProjectedArea(node.aabb); //this function is called with rootProjectedArea < 0 when we want to initialize it

			float projectedArea = influenceArea.getProjectedArea(node.aabb);
			float hitProbability = projectedArea / rootProjectedArea;
			float cost = node.isLeaf() ? 1.2f : 1.0f;
			return hitProbability * node.triangles.size() * cost;
		}

		/**
		 * @brief Given the @p Aabb, it returns an array where each element contains an axis and a function to evaluate.
		 * Axis are sorted from longest to shortest. If an axis is too short, it is not incuded.
		 * The function returns whether it is worth it to try the corresponding axis, given the results obtained with the previous axis.
		 */
		template<float costThreshold, float ratioThreshold = 0.5f>
		static Bvh::ChooseSplittingPlanesReturnType chooseSplittingPlanesLongest(const Bvh::Node& node, const InfluenceArea&, Axis, std::mt19937&) {
			using namespace std;
			array<tuple<float, Axis>, 3> axisLengths{ tuple{node.aabb.size().x, Axis::X}, {node.aabb.size().y, Axis::Y} , {node.aabb.size().z, Axis::Z} }; //basically a dictionary<length, Axis>
			sort(axisLengths.begin(), axisLengths.end(), [](auto a, auto b) { return get<0>(a) < get<0>(b); }); //sort based on axis length

			vector<tuple<Axis, function<bool(float)>>> result{}; //the array to fill and return
			float longestAxis = get<0>(axisLengths[0]);
			//basically, for each axis, decide whether to include it or not, and build a function that decides if it is worth it to try the next axis based on the results of the previous one(s)
			for (int i = 0; const auto & [length, axis] : axisLengths) {
				float ratio = length / longestAxis;
				if (ratio > ratioThreshold) {
					result.emplace_back(
						axis,
						[i](float bestCostSoFar) { return bestCostSoFar > costThreshold + 0.1f * i * costThreshold; } //it suggests to try this axis if the found cost is > of a user-defined threshold plus a percentage (based on how many axis we've already analyzed)
					);
					i++; //counts how many axis we analyzed
					continue;
				}
				break; //if we haven't entered the if, it means all next axis won't enter it either
			}

			return result;
		}

		/**
		 * @brief Returns a list of axis and satisfaction criteria predicates.
		 * Axis are chosen based on the direction of the rays. It tries to avoid planes that are perpendicular to the rays, in order to try and minimize intersections.
		 * If the main rays direction and the best plane considering this direction are "clear", it will return just one plane; else it will return the best 2 planes.
		 * The satisfaction criteria will determine whether it is worth it to try the second plane or not, based on the results of the first split.
		 *
		 * @tparam costThreshold The maximum cost after the first plane split NOT to try the second plane split.
		 * @tparam percentageMargin The margin, in percent points, between axis directions to determine if a direction is "clear". e.g. If the rays have direction <-0.66, 0, 0.75> and the margin is 10%, then the main direction is not clear, because |-0.66|/(|-0.66|+|0.75|) = 47%, and |0.75|/(|-0.66|+|0.75|) = 53% and 53%-47% = 6% < 10%
		 */
		template<float costThreshold, float percentageMargin = 0.05f>
		static Bvh::ChooseSplittingPlanesReturnType chooseSplittingPlanesFacing(const Bvh::Node& node, const InfluenceArea& influenceArea, Axis, std::mt19937& rng) {
			using namespace std;
			using namespace utilities;

			Vector3 dir = influenceArea.getRayDirection(node.aabb); //TODO this will not work for point influence areas, we need to think about them
			float xAbs = abs(dir.x), yAbs = abs(dir.y), zAbs = abs(dir.z);
			Vector3 percs{ xAbs / (xAbs + yAbs + zAbs), yAbs / (xAbs + yAbs + zAbs), zAbs / (xAbs + yAbs + zAbs) };

			//given 2 axis, returns their relative percentages (in the same order as the arguments)
			auto remaining2AxisPercentages = [dir](Axis axis1, Axis axis2) {
				float a1 = abs(at(dir, axis1));
				float a2 = abs(at(dir, axis2));

				if (a1 + a2 < TOLERANCE) return tuple{ 0.5f, 0.5f }; // avoid division by 0

				float a1Perc = a1 / (a1 + a2);
				float a2Perc = a2 / (a1 + a2);

				return tuple{ a1Perc, a2Perc };
				};

			//adds the planes to the selectable planes list. Look at the flowchart to understand the code.
			auto addPlanes = [&remaining2AxisPercentages](Axis a1, float pa1, Axis a2, float pa2, float margin) {
				vector<tuple<Axis, function<bool(float)>>> result{};

				auto [b11, b12] = other2(a1); //if the main direction of the rays is X, we want to cut with planes whose perpendicular is Y or Z (so that we will have fewer intersections)
				auto [pb11, pb12] = remaining2AxisPercentages(b11, b12); //for these 2 axis, get the percentages
				//b11 is the plane whose perpendicular is facing the main rays direction the least (the best possible axis aligned division plane)
				if (pb11 > pb12) {
					std::swap(b11, b12);
					std::swap(pb11, pb12);
				}
				//add the best possible plane
				result.emplace_back(b11, [](float bestCostSoFar) { return true; }); //always try the first plane

				//if the main rays direction is "clear", but the best plane is not so "clear", we also add the other plane considering the "clear" main rays direction
				if (pa1 - margin > pa2 && pb12 - margin <= pb11) {
					//add the other plane with this main rays direction too
					result.emplace_back(b12, [](float bestCostSoFar) { return bestCostSoFar > costThreshold; }); //try this plane if the results with the first plane were worse than the threshold
				}
				//if the main rays direction is not "clear", let's get the second main direction
				else if (pa1 - margin <= pa2) {
					auto [b21, b22] = other2(a2);
					auto [pb21, pb22] = remaining2AxisPercentages(b21, b22);
					//b21 is the plane whose perpendicular is facing the second main rays direction the least 
					if (pb21 > pb22) {
						std::swap(b21, b22);
						std::swap(pb21, pb22);
					}

					//if the best possible plane in the second main rays direction is NOT the same as the best plane in the main rays direction, add it
					if (b21 != b11) {
						result.emplace_back(b21, [](float bestCostSoFar) { return bestCostSoFar > costThreshold; }); //try this plane if the results with the first plane were worse than the threshold
					}
					//if the best possible plane in the second main rays direction is the same as the best plane in the main rays direction, verify if also the other plane is "good enough"
					else if (pb22 - margin <= pb21) {
						result.emplace_back(b22, [](float bestCostSoFar) { return bestCostSoFar > costThreshold; }); //try this plane if the results with the first plane were worse than the threshold
					}
				}

				//at the end of the day, if the main rays direction and the best plane considering this direction is "clear", we will have just one plane; else we will have the best 2 planes.
				//the satisfaction criteria will determine whether it is worth it to try the second plane or not, based on the results of the first split.
				return result;
				};

			Axis max = percs.x >= percs.y && percs.x >= percs.z ? Axis::X : percs.y >= percs.x && percs.y >= percs.z ? Axis::Y : Axis::Z;
			Axis min = percs.x < percs.y && percs.x < percs.z ? Axis::X : percs.y <= percs.x && percs.y <= percs.z ? Axis::Y : Axis::Z; //the first comparison is < and not <= so that if all axis are equal, then max != min
			Axis mid = third(max, min);

			return addPlanes(max, at(percs, max), mid, at(percs, mid), percentageMargin);
		}



		/**
		 * @brief Returns true if the max level has been passed or if the cost of the leaf is low enough.
		 */
		static Bvh::ShouldStopReturnType shouldStopThresholdOrLevel(const Bvh::Node& node, const Bvh::Properties& properties, int currentLevel, float nodeCost) {
			return currentLevel > properties.maxLevels || nodeCost < properties.maxLeafCost || node.triangles.size() < properties.maxTrianglesPerLeaf;
		}
	}
}
