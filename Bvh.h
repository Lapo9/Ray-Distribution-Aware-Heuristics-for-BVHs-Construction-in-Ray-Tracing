#include <glm/glm.hpp>
#include <vector>
#include <memory>
#include <functional>
#include <array>
#include <algorithm>
#include "InfluenceArea.h"

namespace pah {
	using Vector3 = glm::vec3;
	using Vector2 = glm::vec2;
	using namespace std;

	struct Triangle {
		Vector3 v1, v2, v3;

		Vector3 center() {
			return (v1 + v2 + v3) / 3.0f;
		}
	};

	class Bvh {
	public:
		enum class Axis {
			X, Y, Z, None
		};

		struct Aabb {
			Vector3 min;
			Vector3 max;
			
			Aabb(){}

			/**
			 * @brief Creates the tightest possible axis aligned bounding box for the given list of triangles.
			 */
			Aabb(vector<Triangle*> triangles) : min{}, max{} {
				for (const auto& t : triangles) {
					if (t->v1.x < min.x) min.x = t->v1.x;
					if (t->v2.x < min.x) min.x = t->v2.x;
					if (t->v3.x < min.x) min.x = t->v3.x;

					if (t->v1.y < min.y) min.y = t->v1.y;
					if (t->v2.y < min.y) min.y = t->v2.y;
					if (t->v3.y < min.y) min.y = t->v3.y;

					if (t->v1.z < min.z) min.z = t->v1.z;
					if (t->v2.z < min.z) min.z = t->v2.z;
					if (t->v3.z < min.z) min.z = t->v3.z;


					if (t->v1.x > max.x) max.x = t->v1.x;
					if (t->v2.x > max.x) max.x = t->v2.x;
					if (t->v3.x > max.x) max.x = t->v3.x;
								 		
					if (t->v1.y > max.y) max.y = t->v1.y;
					if (t->v2.y > max.y) max.y = t->v2.y;
					if (t->v3.y > max.y) max.y = t->v3.y;
								 		
					if (t->v1.z > max.z) max.z = t->v1.z;
					if (t->v2.z > max.z) max.z = t->v2.z;
					if (t->v3.z > max.z) max.z = t->v3.z;
				}
			}

			/**
			 * @brief Returns the center of the AABB.
			 */
			Vector3 center() const {
				return (min + max) / 2.0f;
			}

			/**
			 * @brief Returns the dimension of this AABB.
			 */
			Vector3 size() const {
				return (max - min);
			}

			float surfaceArea() const {
				return 2.0f * (
					(max.x - min.x) * (max.y - min.y) +
					(max.x - min.x) * (max.z - min.z) +
					(max.y - min.y) * (max.z - min.z));
			}

			float minAxis(Axis axis) const {
				return axis == Axis::X ? min.x : axis == Axis::Y ? min.y : axis == Axis::Z ? min.z : numeric_limits<float>::quiet_NaN();
			}

			float maxAxis(Axis axis) const {
				return axis == Axis::X ? max.x : axis == Axis::Y ? max.y : axis == Axis::Z ? max.z : numeric_limits<float>::quiet_NaN();
			}

			static Aabb maxAabb() {
				Aabb res;
				res.min = Vector3{ numeric_limits<float>::min() };
				res.max = Vector3{ numeric_limits<float>::max() };
				return res;
			}
		};

		struct Node {
			Aabb aabb;
			unique_ptr<Node> leftChild;
			unique_ptr<Node> rightChild;
			vector<Triangle*> triangles;

			Node(Aabb aabb) : aabb{ aabb } {};

			Node(const vector<Triangle*>& triangles) {
				aabb = Aabb{ triangles };
				this->triangles = triangles;
			}

			Node() = default;
			Node(Node&&) = default;
			Node& operator=(Node&&) = default;
			Node(const Node&) = delete;
			Node& operator=(const Node&) = delete;

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
			int maxLevels;
			int bins;

			Properties(const Properties&) = default;
		};

		//custom types
		using ComputeCostReturnType = float;																using ComputeCostType = function<ComputeCostReturnType(const Node&)>;
		using ChooseSplittingPlanesReturnType = vector<tuple<Axis, function<bool(float bestCostSoFar)>>>;	using ChooseSplittingPlanesType = function<ChooseSplittingPlanesReturnType(const Aabb&, const InfluenceArea&, Axis father)>;
		using ShouldStopReturnType = bool;																	using ShouldStopType = function<ShouldStopReturnType(const Properties&, const Node&, int currentLevel, float nodeCost)>;


		Bvh(const Properties&, const InfluenceArea&, const ComputeCostType& computeCost, const ChooseSplittingPlanesType& chooseSplittingPlanes, const ShouldStopType& shouldStop);

		void build(const vector<Triangle*>& triangles);

	private:
		void splitNode(Node& node, Axis fathersplittingAxis, int currentLevel = 0);

		const Node& getRoot() const;

		/**
		 * @brief Given a list of triangles, an axis and a position on this axis, returns 2 sets of triangles, the ones "to the left" of the plane, and the ones "to the right".
		 */
		static tuple<vector<Triangle*>, vector<Triangle*>> splitTriangles(const vector<Triangle*>& triangles, Axis axis, float splittingPlanePosition) {
			//this function returns wether a triangle is positioned "before" a certain plane
			auto triangleLessThan = [](Triangle& t, Axis axis, float pos) {
				if (axis == Axis::X) return t.center().x < pos;
				else if (axis == Axis::Y) return t.center().y < pos;
				else return t.center().z < pos;
			};

			vector<Triangle*> left, right;

			for (auto t : triangles) {
				if (triangleLessThan(*t, axis, splittingPlanePosition)) left.push_back(t);
				else right.push_back(t);
			}

			return { left, right };
		}

		/**
		 * @brief Computes the surface area heuristic of the specified node of a BVH whose root has surface area @p rootSurfaceArea.
		 */
		static ComputeCostReturnType computeCostSah(const Node& node, float rootSurfaceArea) {
			float hitProbability = node.aabb.surfaceArea() / rootSurfaceArea;
			float cost = node.isLeaf() ? 1.2f : 1.0f;
			return hitProbability * node.triangles.size() * cost;
		}

		static ComputeCostReturnType computeCostPah(const Node& node, float rootSurfaceArea) {
			//TODO implement after having defined InfluenceArea
			return -1;
		}
		
		/**
		 * @brief Given the AABB, it returns an array where each element contains an axis and a function to evaluate.
		 * Axis are sorted from longest to shortest. If an axis is too short, it is not incuded. 
		 * The function returns whether it is worth it to try the corresponding axis, given the results obtained with the previous axis.
		 */
		static ChooseSplittingPlanesReturnType chooseSplittingPlanesLongest(const Aabb& aabb, const InfluenceArea&, Axis) {
			array<tuple<float, Axis>, 3> axisLengths{ tuple{aabb.size().x, Axis::X}, {aabb.size().y, Axis::Y} , {aabb.size().z, Axis::Z} }; //basically a dictionary<length, Axis>
			sort(axisLengths.begin(), axisLengths.end(), [](auto a, auto b) {get<0>(a) < get<0>(b); }); //sort based on axis length
			
			vector<tuple<Axis, function<bool(float)>>> result; //the array to fill and return
			float longestAxis = get<0>(axisLengths[0]);
			//basically, for each axis, decide whether to include it or not, and build a function that decides if it is worth it to try the next axis based on the results of the previous one(s)
			for (const auto& [length, axis] : axisLengths) {
				float ratio = length / longestAxis;
				if (ratio > 0.5f) { //TODO 0.5 is arbitrary
					result.emplace_back(tuple{ axis, [ratio](float bestCostSoFar) {return bestCostSoFar / 100.0f > ratio; } }); //TODO the lambda must be reviewed, it is almost a placeholder now
				}
			}

			return result;
		}

		static ChooseSplittingPlanesReturnType chooseSplittingPlanesFacing(const Aabb& aabb, const InfluenceArea& influenceArea, Axis) {
			//TODO implement
			return vector<tuple<Axis, function<bool(float bestCostSoFar)>>>{};
		}

		/**
		 * @brief Returns true if the max level has been passed or if the cost of the leaf is low enough.
		 */
		static ShouldStopReturnType shouldStopThresholdOrLevel(const Properties& properties, const Node&, int currentLevel, float nodeCost) {
			return currentLevel > properties.maxLevels || nodeCost > properties.maxLeafCost;
		}


		Node root;
		Properties properties;
		InfluenceArea influenceArea;

		//customizable functions
		ComputeCostType computeCost;

		//the idea is that this function returns a list of suitable axis to try to subdivide the AABB into, and a set of corresponding predicates.
		//these predicates take into account the "quality" of the axis, and the results obtained from previous axis; and they decide whether is it worth it to try the next axis
		ChooseSplittingPlanesType chooseSplittingPlanes;
		ShouldStopType shouldStop;
	};
}
