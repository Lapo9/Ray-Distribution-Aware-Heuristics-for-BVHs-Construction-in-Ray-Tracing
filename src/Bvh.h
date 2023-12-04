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

namespace pah {
	class Bvh {

		#ifdef DEBUG
		#define DBG(x)
		#else
		#define DBG(x) x
		#endif // DEBUG


	public:
		struct NodeTimingInfo {
			using DurationNs = chrono::duration<long long, std::nano>;
			DurationNs total;
			DurationNs splittingTot;
			DurationNs computeCostTot;
			int computeCostCount;
			DurationNs splitTrianglesTot;
			int splitTrianglesCount;
			DurationNs chooseSplittingPlanesTot;
			int chooseSplittingPlanesCount;
			DurationNs shouldStopTot;
			int shouldStopCount;

			DurationNs computeCostMean() { return computeCostTot / computeCostCount; }
			DurationNs splitTrianglesMean() { return splitTrianglesTot / splitTrianglesCount; }
			DurationNs chooseSplittingPlanesMean() { return chooseSplittingPlanesTot / chooseSplittingPlanesCount; }
			DurationNs shouldStopMean() { return shouldStopTot / shouldStopCount; }
			
			void logComputeCost(DurationNs duration) {
				computeCostTot += duration;
				computeCostCount++;
			}

			void logSplitTriangles(DurationNs duration) {
				splitTrianglesTot += duration;
				splitTrianglesCount++;
			}

			void logChooseSplittingPlanes(DurationNs duration) {
				chooseSplittingPlanesTot += duration;
				chooseSplittingPlanesCount++;
			}

			void logShouldStop(DurationNs duration) {
				shouldStopTot += duration;
				shouldStopCount++;
			}
		};

		struct Node {
			Aabb aabb;
			unique_ptr<Node> leftChild;
			unique_ptr<Node> rightChild;
			vector<const Triangle*> triangles;
			DBG(NodeTimingInfo nodeTimingInfo;)

			Node(Aabb aabb) : aabb{ aabb } {};

			Node(const vector<const Triangle*>& triangles) : aabb{ Aabb{triangles} }, triangles{ triangles } {}

			Node() = default;
			Node(Node&&) = default;
			Node& operator=(Node&&) = default;
			Node(const Node&) = delete;
			Node& operator=(const Node&) = delete;
			~Node() = default;

			bool isLeaf() const {
				return leftChild == nullptr && rightChild == nullptr;
			}
		};

		enum class Heuristic {
			SAH, PAH, RANDOM
		};

		enum class SplitPlaneStrategy {
			LONGEST, PLANE_FACING, ROUND_ROBIN, RANDOM
		};

		struct Properties {
			Heuristic heuristic;
			SplitPlaneStrategy splitPlaneStrategy;
			float maxLeafCost;
			int maxTrianglesPerLeaf;
			int maxLevels;
			int bins;

			Properties() = default;
			Properties(const Properties&) = default;
			Properties& operator=(const Properties&) = default;
		};

		//custom types
		using ComputeCostReturnType = float;																using ComputeCostType = ComputeCostReturnType(const Node&, const InfluenceArea&, float rootMetric);
		using ChooseSplittingPlanesReturnType = vector<tuple<Axis, function<bool(float bestCostSoFar)>>>;	using ChooseSplittingPlanesType = ChooseSplittingPlanesReturnType(const Aabb&, const InfluenceArea&, Axis father, mt19937& rng);
		using ShouldStopReturnType = bool;																	using ShouldStopType = ShouldStopReturnType(const Properties&, const Node&, int currentLevel, float nodeCost);


		Bvh(const Properties&, const InfluenceArea&, ComputeCostType computeCost, ChooseSplittingPlanesType chooseSplittingPlanes, ShouldStopType shouldStop);

		friend bool operator==(const Bvh& bvh1, const Bvh& bvh2) {
			return bvh1.id == bvh2.id;
		}

		void build(const vector<Triangle>& triangles);
		void build(const vector<Triangle>& triangles, unsigned int seed);

		const Node& getRoot() const;
		const InfluenceArea& getInfluenceArea() const;


		/**
		 * @brief Computes the surface area heuristic of the specified node of a BVH whose root has surface area @p rootSurfaceArea.
		 */
		static ComputeCostReturnType computeCostSah(const Node& node, const InfluenceArea&, float rootArea) {
			if (rootArea < 0) return node.aabb.surfaceArea(); //this function is called with rootArea < 0 when we want to initialize it

			float hitProbability = node.aabb.surfaceArea() / rootArea;
			float cost = node.isLeaf() ? 1.2f : 1.0f;
			return hitProbability * node.triangles.size() * cost;
		}

		static ComputeCostReturnType computeCostPah(const Node& node, const InfluenceArea& influenceArea, float rootProjectedArea) {
			if (rootProjectedArea < 0) return influenceArea.getProjectedArea(node.aabb); //this function is called with rootProjectedArea < 0 when we want to initialize it

			float projectedArea = influenceArea.getProjectedArea(node.aabb);
			float hitProbability = projectedArea / rootProjectedArea;
			float cost = node.isLeaf() ? 1.2f : 1.0f;
			return hitProbability * node.triangles.size() * cost;
		}

		/**
		 * @brief Given the AABB, it returns an array where each element contains an axis and a function to evaluate.
		 * Axis are sorted from longest to shortest. If an axis is too short, it is not incuded.
		 * The function returns whether it is worth it to try the corresponding axis, given the results obtained with the previous axis.
		 */
		static ChooseSplittingPlanesReturnType chooseSplittingPlanesLongest(const Aabb& aabb, const InfluenceArea&, Axis, mt19937&) {
			array<tuple<float, Axis>, 3> axisLengths{ tuple{aabb.size().x, Axis::X}, {aabb.size().y, Axis::Y} , {aabb.size().z, Axis::Z} }; //basically a dictionary<length, Axis>
			sort(axisLengths.begin(), axisLengths.end(), [](auto a, auto b) { return get<0>(a) < get<0>(b); }); //sort based on axis length

			vector<tuple<Axis, function<bool(float)>>> result{}; //the array to fill and return
			float longestAxis = get<0>(axisLengths[0]);
			//basically, for each axis, decide whether to include it or not, and build a function that decides if it is worth it to try the next axis based on the results of the previous one(s)
			for (const auto& [length, axis] : axisLengths) {
				float ratio = length / longestAxis;
				if (ratio > 0.5f) { //TODO 0.5 is arbitrary
					result.emplace_back(axis, [](float bestCostSoFar) {return bestCostSoFar / 100.0f > 1; }); //TODO the lambda must be reviewed, it is almost a placeholder now
				}
			}

			return result;
		}

		static ChooseSplittingPlanesReturnType chooseSplittingPlanesFacing(const Aabb& aabb, const InfluenceArea& influenceArea, Axis, mt19937& rng) {
			using namespace utilities;
			Vector3 dir = influenceArea.getRayDirection(aabb); //TODO this will not work for point influence areas, we need to think about them

			Vector3 percs{ dir.x / (dir.x + dir.y + dir.z),  dir.y / (dir.x + dir.y + dir.z), dir.z / (dir.x + dir.y + dir.z) };

			uniform_real_distribution extractUniform(0.0f, 1.0f); //uniform distribution between 0 and 1 from which to extract random numbers

			//given 2 axis, returns their relative percentages (in the same order as the arguments)
			auto remaining2AxisPercentages = [dir](Axis axis1, Axis axis2) {
				float a1 = at(dir, axis1);
				float a2 = at(dir, axis2);

				float a1Perc = a1 / (a1 + a2);
				float a2Perc = a2 / (a2 + a1);

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
				result.emplace_back(b11, [](float bestCostSoFar) { return true; }); //TODO satisfaction criterium

				//if the main rays direction is "clear", but the best plane is not so "clear", we also add the other plane considering the "clear" main rays direction
				if (pa1 - margin > pa2 && pb11 - margin <= pb12) {
					//add the other plane with this main rays direction too
					result.emplace_back(b12, [](float bestCostSoFar) { return true; }); //TODO satisfaction criterium
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
						result.emplace_back(b21, [](float bestCostSoFar) { return true; }); //TODO satisfaction criterium
					}
					//if the best possible plane in the second main rays direction is the same as the best plane in the main rays direction, verify if also the othe plane is "good enough"
					else if (pb21 - margin <= pb22) {
						result.emplace_back(b22, [](float bestCostSoFar) { return true; }); //TODO satisfaction criterium
					}
				}

				//at the end of the day, if the main rays direction and the best plane considering this direction is "clear", we will have just one plane; else we will have the best 2 planes.
				//the satisfaction criteria will decide whether is it worth it to try the second plane or not, based on the results of the first split.
				return result;
				};

			Axis max = percs.x >= percs.y && percs.x >= percs.z ? Axis::X : percs.y >= percs.x && percs.y >= percs.z ? Axis::Y : Axis::Z;
			Axis min = percs.x < percs.y && percs.x < percs.z ? Axis::X : percs.y <= percs.x && percs.y <= percs.z ? Axis::Y : Axis::Z; //the first comparison is < and not <= so that if all axis are equal, then max != min
			Axis mid = third(max, min);

			return addPlanes(max, at(percs, max), mid, at(percs, mid), 0.05f);
		}

		/**
		 * @brief Returns true if the max level has been passed or if the cost of the leaf is low enough.
		 */
		static ShouldStopReturnType shouldStopThresholdOrLevel(const Properties& properties, const Node& node, int currentLevel, float nodeCost) {
			return currentLevel > properties.maxLevels || nodeCost < properties.maxLeafCost || node.triangles.size() < properties.maxTrianglesPerLeaf;
		}

	private:
		void splitNode(Node& node, Axis fathersplittingAxis, int currentLevel);

		//simple wrappers for the custom functions. We use wrappers because there may be some common actions to perform before (e.g. time logging)
		ComputeCostReturnType computeCostWrapper(const Node& node, const InfluenceArea& influenceArea, float rootArea);
		ChooseSplittingPlanesReturnType chooseSplittingPlaneWrapper(const Aabb& aabb, const InfluenceArea& influenceArea, Axis axis, mt19937& rng);
		ShouldStopReturnType shouldStopWrapper(const Properties& properties, const Node& node, int currentLevel, float nodeCost);

		/**
		 * @brief Given a list of triangles, an axis and a position on this axis, returns 2 sets of triangles, the ones "to the left" of the plane, and the ones "to the right".
		 */
		static tuple<vector<const Triangle*>, vector<const Triangle*>> splitTriangles(const vector<const Triangle*>& triangles, Axis axis, float splittingPlanePosition) {
			using namespace utilities;
			vector<const Triangle*> left, right;

			for (auto t : triangles) {
				if (at(t->center(), axis) < splittingPlanePosition) left.push_back(t);
				else right.push_back(t);
			}

			return { left, right };
		}


		Node root;
		float rootMetric; //stores the cost metric of the root (e.g. surface area if we use SAH, projected area if we use PAH, ...)
		Properties properties;
		const InfluenceArea* influenceArea;

		//customizable functions
		function<ComputeCostType> computeCost;

		//the idea is that this function returns a list of suitable axis to try to subdivide the AABB into, and a set of corresponding predicates.
		//these predicates take into account the "quality" of the axis, and the results obtained from previous axis; and they decide whether is it worth it to try the next axis
		function<ChooseSplittingPlanesType> chooseSplittingPlanes;
		function<ShouldStopType> shouldStop;

		mt19937 rng; //random number generator
		unsigned long long int id; //id of this BVH: it is a cheap way to check if 2 BVHs are equals
	};
}
