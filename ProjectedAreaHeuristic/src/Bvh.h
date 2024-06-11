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

			Node(const Node& orig) :
				aabb{ orig.aabb },
				triangles{ orig.triangles },
				TIME(nodeTimingInfo{ orig.nodeTimingInfo }),
				leftChild{ orig.leftChild != nullptr ? new Node(*orig.leftChild) : nullptr },
				rightChild{ orig.rightChild != nullptr ? new Node(*orig.rightChild) : nullptr } {
			}

			Node& operator=(const Node& orig) {
				aabb = orig.aabb;
				triangles = orig.triangles;
				TIME(nodeTimingInfo = orig.nodeTimingInfo);
				leftChild = orig.leftChild != nullptr ? std::make_unique<Node>(*orig.leftChild) : nullptr;
				rightChild = orig.rightChild != nullptr ? std::make_unique<Node>(*orig.rightChild) : nullptr;
				return *this;
			}

			Node(Node&& orig) :
				aabb{ std::move(orig.aabb) },
				triangles{ std::move(orig.triangles) },
				TIME(nodeTimingInfo{ std::move(orig.nodeTimingInfo) }),
				leftChild{ std::move(orig.leftChild) },
				rightChild{ std::move(orig.rightChild) } {
			}

			Node& operator=(Node&& orig) {
				aabb = std::move(orig.aabb);
				triangles = std::move(orig.triangles);
				TIME(nodeTimingInfo = std::move(orig.nodeTimingInfo));
				leftChild = std::move(orig.leftChild);
				rightChild = std::move(orig.rightChild);
				return *this;
			}

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
			const Bvh* bvh;
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
			float maxLeafArea; /**< If a @p Node area is less than this threshold, it is automatically a leaf. */
			float maxLeafHitProbability; /**< If a @p Node hit probability is less than this threshold, it is automatically a leaf. */
			int maxTrianglesPerLeaf; /**< If a @p Node has less triangles than this threshold, it is automatically a leaf. */
			int maxLevels; /**< Max number of levels of the @p Bvh. */
			int bins; /**< How many bins are used to create the nodes of th @p Bvh. The higher, the better the @p Bvh, but the slower the construction. */
			int maxNonFallbackLevels; /**< Nodes after this level will use the fallback (SAH) method. */
			float splitPlaneQualityThreshold;
			float acceptableChildrenFatherHitProbabilityRatio;
			float excellentChildrenFatherHitProbabilityRatio;
		};

		//custom alias
		using ComputeCostReturnType = struct { float cost, hitProbability, area; };		using ComputeCostType = ComputeCostReturnType(const Node&, const InfluenceArea*, float rootMetric);
		using ChooseSplittingPlanesReturnType = std::vector<std::pair<Axis, float>>;	using ChooseSplittingPlanesType = ChooseSplittingPlanesReturnType(const Node&, const InfluenceArea*, Axis father, std::mt19937& rng);
		using ShouldStopReturnType = bool;												using ShouldStopType = ShouldStopReturnType(const Node&, const Properties&, int currentLevel, const ComputeCostReturnType& nodeCost);


		Bvh(const Properties&, const InfluenceArea&, ComputeCostType computeCost, ChooseSplittingPlanesType chooseSplittingPlanes, ShouldStopType shouldStop, std::string name);

		Bvh(const Properties&, ComputeCostType computeCost, ChooseSplittingPlanesType chooseSplittingPlanes, ShouldStopType shouldStop, std::string name);

		/**
		 * @brief Returns whether @p bvh1 is the same as @p bvh2.
		 * To do so, it compares the ids of the 2 @p Bvh s. This is just a cheap way to understand if we are dealing with the exact same object.
		 * The id changes every time the @p Bvh is built.
		 */
		friend bool operator==(const Bvh& bvh1, const Bvh& bvh2) {
			return bvh1.id == bvh2.id;
		}

		/**
		 * @brief Changes the fallback compute cost strategy.
		 */
		void setFallbackComputeCostStrategy(ComputeCostType computeCostFallback);
		/**
		 * @brief Changes the fallback choose splitting plane strategy.
		 */
		void setFallbackChooseSplittingPlaneStrategy(ChooseSplittingPlanesType chooseSplittingPlaneFallback);
		/**
		 * @brief Changes the fallback should stop strategy.
		 */
		void setFallbackShouldStopStrategy(ShouldStopType shouldStopFallback);

		/**
		 * @brief Constructs the @p Bvh on a set of triangles. The seed for the random operations during the construction is random.
		 * 
		 * @param splitPlaneQualityThreshold How low the quality of the split plane can be before falling back on the standars SAH methods to compute the cost and splits. The value can be between 0 (bad) and 1 (good).
		 * @param maxChildrenFatherHitProbabilityRatio (rightChildHitProbability + leftChildHitProbability) / fatherHitProbability: how big this ratio can be to consider a split acceptable.
		 */
		void build(const std::vector<Triangle>& triangles);
		void build(const std::vector<const Triangle*>& triangles);

		/**
		 * @brief Constructs the @p Bvh on a set of triangles. It is possible to specify the seed for the random operations during the construction.
		 * 
		 * @param splitPlaneQualityThreshold How low the quality of the split plane can be before falling back on the standars SAH methods to compute the cost and splits. The value can be between 0 (bad) and 1 (good).
		 * @param maxChildrenFatherHitProbabilityRatio (rightChildHitProbability + leftChildHitProbability) / fatherHitProbability: how big this ratio can be to consider a split acceptable.
		 */
		void build(const std::vector<Triangle>& triangles, unsigned int seed);
		void build(const std::vector<const Triangle*>& triangles, unsigned int seed);

		/**
		 * @brief Traverses the @p Bvh and returns some stats about the traversal.
		 */
		TraversalResults traverse(const Ray& ray) const;


		const Node& getRoot() const; /**< @brief Returns the root of the @p Bvh. */
		const InfluenceArea* getInfluenceArea() const; /**< @brief Returns the @p InfluenceArea of the @p Bvh. */
		INFO(const DurationMs getTotalBuildTime() const;); /**< @brief Returns the time it took to build this @p Bvh. */
		const Properties getProperties() const; /**< @brief Returns the properties of this @p Bvh. */

		const std::string name;
	private:
		/**
		 * @brief Given a @p Node, it splits it into 2 children according to the strategies set during @p Bvh construction.
		 */
		void splitNode(Node& node, Axis fatherSplittingAxis, float fatherHitProbability, int currentLevel);

		//simple wrappers for the custom functions. We use wrappers because there may be some common actions to perform before (e.g. time logging)
		ComputeCostReturnType computeCostWrapper(const Node& parent, const Node& node, const InfluenceArea* influenceArea, float rootArea, int level, bool forceSah = false);
		ChooseSplittingPlanesReturnType chooseSplittingPlanesWrapper(const Node& node, const InfluenceArea* influenceArea, Axis axis, std::mt19937& rng, int level, bool forceSah = false);
		ShouldStopReturnType shouldStopWrapper(const Node& parent, const Node& node, const Properties& properties, int currentLevel, const ComputeCostReturnType& nodeCost, int level, bool forceSah = false);

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
		std::function<ComputeCostType> computeCostFallback;
		//the idea is that this function returns a list of suitable axis to try to subdivide the AABB into, and a set of corresponding predicates.
		//these predicates take into account the "quality" of the axis, and the results obtained from previous axis; and they decide whether is it worth it to try the next axis
		std::function<ChooseSplittingPlanesType> chooseSplittingPlanes;
		std::function<ChooseSplittingPlanesType> chooseSplittingPlanesFallback;
		std::function<ShouldStopType> shouldStop;
		std::function<ShouldStopType> shouldStopFallback;

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
		static Bvh::ComputeCostReturnType computeCostSah(const Bvh::Node& node, const InfluenceArea*, float rootArea) {
			float cost = node.isLeaf() ? LEAF_COST : NODE_COST;
			//this function is called with rootArea < 0 when we want to initialize it
			if (rootArea < 0) return { node.aabb.surfaceArea() * node.triangles.size() * cost, 1, node.aabb.surfaceArea()};

			float surfaceArea = node.aabb.surfaceArea();
			float hitProbability = glm::min(surfaceArea / rootArea, 1.0f);
			return { hitProbability * node.triangles.size() * cost, hitProbability, surfaceArea };
		}

		/**
		 * @brief Computes the projected area heuristic of the specified node of a @p Bvh whose @p InfluenceArea has @p rootSurfaceArea.
		 */
		static Bvh::ComputeCostReturnType computeCostPah(const Bvh::Node& node, const InfluenceArea* influenceArea, float rootProjectedArea) {
			float cost = node.isLeaf() ? LEAF_COST : NODE_COST;
			//this function is called with rootProjectedArea < 0 when we want to initialize it
			//TODO test if this work (maybe let the user choose)
			if (rootProjectedArea < 0) return { influenceArea->getProjectionPlaneArea() * node.triangles.size() * cost, 1, influenceArea->getProjectionPlaneArea() };

			float projectedArea = influenceArea->getProjectedArea(node.aabb);
			float hitProbability = glm::min(projectedArea / rootProjectedArea, 1.f);
			return { hitProbability * node.triangles.size() * cost, hitProbability, projectedArea };
		}

		/**
		 * @brief Computes the projected area heuristic of the specified node of a @p Bvh whose @p InfluenceArea has surface area @p rootSurfaceArea.
		 * It uses culling to compute the area of the node that is actually inside the @p InfluenceArea projection plane.
		 */
		static Bvh::ComputeCostReturnType computeCostPahWithCulling(const Bvh::Node& node, const InfluenceArea* influenceArea, float rootProjectedArea) {
			float cost = node.isLeaf() ? LEAF_COST : NODE_COST;
			//this function is called with rootProjectedArea < 0 when we want to initialize it
			if (rootProjectedArea < 0) return { influenceArea->getProjectionPlaneArea() * node.triangles.size() * cost, 1, influenceArea->getProjectionPlaneArea() };

			float projectedArea = overlappingArea(ConvexHull2d{ influenceArea->getProjectedHull(node.aabb) }, ConvexHull2d{ influenceArea->getProjectionPlaneHull() });
			float hitProbability = glm::min(projectedArea / rootProjectedArea, 1.f);
			return { hitProbability * node.triangles.size() * cost, hitProbability, projectedArea };
		}


		/**
		 * @brief Given the @p Aabb, it returns an array where each element contains an axis and a function to evaluate.
		 * Axis are sorted from longest to shortest. If an axis is too short, it is not incuded.
		 * The function returns whether it is worth it to try the corresponding axis, given the results obtained with the previous axis.
		 */
		template<float qualityThreshold = 0.f>
		static Bvh::ChooseSplittingPlanesReturnType chooseSplittingPlanesLongest(const Bvh::Node& node, const InfluenceArea*, Axis, std::mt19937&) {
			using namespace std;
			array<pair<float, Axis>, 3> axisLengths{ tuple{node.aabb.size().x, Axis::X}, {node.aabb.size().y, Axis::Y} , {node.aabb.size().z, Axis::Z} }; //basically a dictionary<length, Axis>
			ranges::sort(axisLengths, [](auto a, auto b) { return a.first > b.first; }); //sort based on axis length

			vector<pair<Axis, float>> result{}; //the array to fill and return
			float longestAxis = get<0>(axisLengths[0]);
			//basically, for each axis, decide whether to include it or not, and add a quality metric for this split axis
			for (int i = 0; const auto & [length, axis] : axisLengths) {
				float ratio = length / longestAxis;
				if (ratio >= qualityThreshold) {
					result.emplace_back(axis, ratio);
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
		 * @tparam maxHitProbabilityRatioWithFather If, after a plane split, the ratio between the sum of the children hit probabilities and the father node hit probability is bigger than this value, a new split will be tried. (e.g. children = 5, father = 10, max = 0.9 ==> not tried because 5/10 < 0.9).
		 * @tparam percentageMargin The margin, in percent points, between axis directions to determine if a direction is "clear". e.g. If the rays have direction <-0.66, 0, 0.75> and the margin is 10%, then the main direction is not clear, because |-0.66|/(|-0.66|+|0.75|) = 47%, and |0.75|/(|-0.66|+|0.75|) = 53% and 53%-47% = 6% < 10%
		 */
		static Bvh::ChooseSplittingPlanesReturnType chooseSplittingPlanesFacing(const Bvh::Node& node, const InfluenceArea* influenceArea, Axis, std::mt19937& rng) {
			using namespace std;
			using namespace utilities;

			Vector3 dir = influenceArea->getRayDirection(node.aabb); //TODO this will not work so well for wide point influence areas
			float xAbs = abs(dir.x), yAbs = abs(dir.y), zAbs = abs(dir.z);
			Vector3 percs{ xAbs / (xAbs + yAbs + zAbs), yAbs / (xAbs + yAbs + zAbs), zAbs / (xAbs + yAbs + zAbs) };

			vector<pair<Axis, float>> result{
				{Axis::X, pow(1.f - percs[0], 2)},
				{Axis::Y, pow(1.f - percs[1], 2)},
				{Axis::Z, pow(1.f - percs[2], 2)}
			};
			ranges::sort(result, [](auto a, auto b) {return a.second > b.second; });
			return result;
		}


		/**
		 * @brief Returns true if the max level has been passed or if the cost of the leaf is low enough.
		 */
		static Bvh::ShouldStopReturnType shouldStopThresholdOrLevel(const Bvh::Node& node, const Bvh::Properties& properties, int currentLevel, const Bvh::ComputeCostReturnType& nodeCost) {
			return currentLevel > properties.maxLevels ||
				nodeCost.cost < properties.maxLeafCost ||
				nodeCost.hitProbability < properties.maxLeafHitProbability ||
				nodeCost.area < properties.maxLeafArea ||
				node.triangles.size() < properties.maxTrianglesPerLeaf;
		}
	}
}
