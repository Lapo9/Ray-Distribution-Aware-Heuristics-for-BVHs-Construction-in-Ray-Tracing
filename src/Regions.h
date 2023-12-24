#pragma once

#include <functional>
#include "glm/glm.hpp"

#include "Utilities.h"

namespace pah {
	//forward declarations
	class Region;
	class Aabb;
	class Obb;
	class AabbForObb;
	namespace collisionDetection {
		static bool areColliding(const Aabb& aabb1, const Aabb& aabb2);
		static bool areColliding(const AabbForObb& aabbForObb, const Aabb& aabb);
		static bool areColliding(const Obb& obb, const Aabb& aabb);
		static bool almostParallel(const Vector3& lhs, const Vector3& rhs, float threshold);
		static bool almostAabb(const Obb& obb);
		static std::pair<int, int> projectedAabbExtremes(const Vector3& axis);
		static std::pair<int, int> projectedObbExtremes(const Vector3& axis, const Obb& obb);
	}



	/**
	 * @brief Base class for anything that can represent a 3D space region.
	 */
	class Region {
	public:
		/**
		 * @brief Returns whether the /p point is inside the region.
		 */
		virtual bool contains(const Vector3& point) const = 0;

		/**
		 * @brief Returns the enclosing /p Aabb.
		 */
		virtual Aabb enclosingAabb() const = 0;

		/**
		 * @brief Returns whether there is a collision between this /p Region and an /p Aabb.
		 * We make this a method in order to make use of the runtime polymorphism features of C++.
		 */
		virtual bool isCollidingWith(const Aabb& aabb) const = 0;

		/**
		 * @brief Returns whether this /p Region fully contains the specified /p Aabb.
		 * We make this a method in order to make use of the runtime polymorphism features of C++.
		 */
		virtual bool fullyContains(const Aabb& aabb) const = 0;
	};

	/**
	 * @brief An axis aligned bounding box.
	 */
	struct Aabb : public Region {
		Vector3 min;
		Vector3 max;

		Aabb() = default;
		/**
		 * @brief Creates the tightest possible axis aligned bounding box for the given list of triangles.
		 */
		Aabb(const std::vector<const Triangle*>& triangles) : min{ std::numeric_limits<float>::max() }, max{ -std::numeric_limits<float>::max() } {
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
		Aabb(const Vector3& min, const Vector3& max) : min{ min }, max{ max } {}

		bool contains(const Vector3& point) const override {
			return
				point.x >= min.x &&
				point.y >= min.y &&
				point.z >= min.z &&
				point.x <= max.x &&
				point.y <= max.y &&
				point.z <= max.z;
		}

		Aabb enclosingAabb() const override {
			return *this;
		}
		
		bool isCollidingWith(const Aabb& aabb) const override {
			return collisionDetection::areColliding(*this, aabb);
		}

		bool fullyContains(const Aabb& aabb) const override {
			return
				min.x <= aabb.min.x &&
				min.y <= aabb.min.y &&
				min.z <= aabb.min.z &&
				max.x >= aabb.max.x &&
				max.y >= aabb.max.y &&
				max.z >= aabb.max.z;
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

		/**
		 * @brief Returns the surface area of the AABB.
		 */
		float surfaceArea() const {
			return 2.0f * (
				(max.x - min.x) * (max.y - min.y) +
				(max.x - min.x) * (max.z - min.z) +
				(max.y - min.y) * (max.z - min.z));
		}

		/**
		 * @brief Given an AABB (min-max form) returns the array of its 8 vertices with this layout:
		 *  2_______________6
		 *  |\              \
		 *  | \______________\7
		 * 0| 3|          4. |
		 *   \ |             |
		 *    \|_____________|
		 *     1             5
		 * See pah::Aabb::getPoint for more info about the logic of this layout.
		 */
		std::array<Vector3, 8> getPoints() const {
			std::array<Vector3, 8> points{};
			for (int i = 0; i < 2; ++i)
				for (int j = 0; j < 2; ++j)
					for (int k = 0; k < 2; ++k) {
						Vector3 vertex{ i, j, k }; //selects which vertex we are considering (e.g. top-left-back)
						points[i * 4 + j * 2 + k] = min + vertex * size();
					}
			return points;
		}

		/**
		 * @brief Starting from the minimum corner (bottom-left-back) we move following this scheme for i:
		 * | rightward |  upward   | frontward |
		 * |     2     |     1     |     0     |
		 *
		 * For example: i = 5 = 101b --> rightward and frontward
		 *     2________6
		 *     |\       |\
		 *     | \        \
		 *     | 3\_____|__\7
		 *    0| -| - - -4  |
		 *     \  |      '  |
		 *      \ |       ' |
		 *       \|________'|
		 *        1         5
		 */
		Vector3 getPoint(int i) const {
			int frontward = (i >> 0) & 1;
			int upward = (i >> 1) & 1;
			int rightward = (i >> 2) & 1;
			return min + (max - min) * Vector3 { rightward, upward, frontward };
		}

		/**
		 * @brief Returns the biggest AABB possible.
		 */
		static Aabb maxAabb() {
			Vector3 min{ -std::numeric_limits<float>::max() };
			Vector3 max{ std::numeric_limits<float>::max() };
			return Aabb{ min, max };
		}

		/**
		 * @brief Returns the biggest negative AABB possible.
		 */
		static Aabb minAabb() {
			Vector3 min{ std::numeric_limits<float>::max() };
			Vector3 max{ -std::numeric_limits<float>::max() };
			return Aabb{ min, max };
		}

		Aabb& operator+=(const Aabb& lhs) {
			min = glm::min(min, lhs.min);
			max = glm::max(max, lhs.max);
			return *this;
		}

		friend Aabb& operator+(Aabb lhs, const Aabb& rhs) {
			return lhs += rhs;
		}
	};

	/**
	 * @brief An oriented bounding box.
	 */
	struct Obb : public Region {
		Vector3 center;
		Vector3 forward;
		Vector3 right;
		Vector3 up;
		Vector3 halfSize;

		Obb(const Vector3& center, const Vector3& halfSize, const Vector3& forward) : center{ center }, halfSize{ halfSize },
			forward{ glm::normalize(forward) }, right{ glm::cross(Vector3{0,1,0}, forward) }, up{ glm::cross(forward, right) } {
		}

		bool contains(const Vector3& point) const override {
			using namespace glm;

			Vector3 d = point - center;
			//we project the distance vector between the point and the center in the reference system of the OBB, and we check the distances are < than the half size in all directions
			return
				abs(dot(d, right)) <= halfSize.x &&
				abs(dot(d, up)) <= halfSize.y &&
				abs(dot(d, forward)) <= halfSize.z;
		}

		Aabb enclosingAabb() const override {
			Aabb aabb{ Vector3{std::numeric_limits<float>::max()}, Vector3{ -std::numeric_limits<float>::max()} };
			auto vertices = getPoints();

			//TODO probably there is a more efficient way
			for (auto& v : vertices) {
				//update max and min of enclosing AABB
				aabb.max = glm::max(aabb.max, v);
				aabb.min = glm::min(aabb.min, v);
			}
			return aabb;
		}

		bool isCollidingWith(const Aabb& aabb) const override {
			return collisionDetection::areColliding(*this, aabb);
		}

		bool fullyContains(const Aabb& aabb) const override {
			const auto& vertices = aabb.getPoints();
			for (const auto& vertex : vertices) {
				if (!contains(vertex)) return false;
			}
			return true;
		}


		/**
		 * @brief Given an OBB returns the array of its 8 vertices with this layout:
		 *  2_______________6
		 *  |\              \
		 *  | \______________\7
		 * 0| 3|          4. |
		 *   \ |             |
		 *    \|_____________|
		 *     1             5
		 * See pah::Aabb::getPoint for more info about the logic of this layout.
		 */
		std::array<Vector3, 8> getPoints() const {
			std::array<Vector3, 8> points{};
			//loop through each vertex of the OBB
			for (int i = -1; i <= 1; i += 2)
				for (int j = -1; j <= 1; j += 2)
					for (int k = -1; k <= 1; k += 2) {
						Vector3 obbVertex = halfSize * Vector3{ i,j,k }; //point in the reference system of the OBB
						Matrix3 rotation{ right, up, forward };
						Vector3 worldVertex = rotation * obbVertex + center; //point in world space
						points[((i + 1) / 2) * 4 + ((j + 1) / 2) * 2 + ((k + 1) / 2) * 1] = worldVertex;
					}
			return points;
		}
	};

	/**
	 * @brief An OBB with an enclosing AABB.
	 */
	struct AabbForObb : public Region {
		Obb obb;
		Aabb aabb;

		AabbForObb(const Vector3& center, const Vector3& halfSize, const Vector3& forward) : obb{ center, halfSize, forward }, aabb{ obb.enclosingAabb() } {}
		AabbForObb(const Obb& obb) : obb{ obb }, aabb{ obb.enclosingAabb() } {}

		bool contains(const Vector3& point) const override {
			//check if the point is inside the AABB (cheap), if it is, check the OBB
			if (aabb.contains(point)) return obb.contains(point);
			return false;
		}

		Aabb enclosingAabb() const override {
			return aabb;
		}

		bool isCollidingWith(const Aabb& aabb) const override {
			return collisionDetection::areColliding(*this, aabb);
		}

		bool fullyContains(const Aabb& aabb) const override {
			if (!this->aabb.fullyContains(aabb)) return false;
			return this->obb.fullyContains(aabb);
		}
	};


	/**
	 * @brief Functions and utilities to detect whether there is a collision between 2 /p Region s.
	 */
	namespace collisionDetection {

		/**
		 * @brief Returns whether 2 /p Aabb s are colliding.
		 */
		static bool areColliding(const Aabb& aabb1, const Aabb& aabb2) {
			return
				aabb1.min.x <= aabb2.max.x &&
				aabb1.max.x >= aabb2.min.x &&
				aabb1.min.y <= aabb2.max.y &&
				aabb1.max.y >= aabb2.min.y &&
				aabb1.min.z <= aabb2.max.z &&
				aabb1.max.z >= aabb2.min.z;
		}

		/**
		 * @brief Implementation of the separating axis theorem between an /p AabbForObb and an /p Aabb.
		 */
		static bool areColliding(const AabbForObb& aabbForObb, const Aabb& aabb) {
			using namespace std;
			using namespace glm;

			//first, we check if the enclosing AABB of the OBB overlaps with the AABB (it can save a lot of time)
			bool aabbsColliding = areColliding(aabb, aabbForObb.aabb);
			if (!aabbsColliding) return false;

			//then we check whether the OBB is "almost" an AABB (in this case we can approximate the collision to the AABB v AABB case)
			const Obb& obb = aabbForObb.obb;
			if (almostAabb(obb)) return aabbsColliding;

			//else, we have to use SAT
			auto obbVertices = obb.getPoints();
			auto aabbVertices = aabb.getPoints();

			//these are the potential separating axes; we use lambdas in order to evaluate them lazily (important to avoid useless cross products in case of early outs)
			auto axes = vector<function<Vector3()>>{
				[]() { return Vector3{1,0,0}; },
				[]() { return Vector3{0,1,0}; },
				[]() { return Vector3{0,0,1}; },
				[&obb]() { return obb.right; },
				[&obb]() { return obb.up; },
				[&obb]() { return obb.forward; },
				[&obb]() { return cross(Vector3{1,0,0}, obb.right); },
				[&obb]() { return cross(Vector3{1,0,0}, obb.up); },
				[&obb]() { return cross(Vector3{1,0,0}, obb.forward); },
				[&obb]() { return cross(Vector3{0,1,0}, obb.right); },
				[&obb]() { return cross(Vector3{0,1,0}, obb.up); },
				[&obb]() { return cross(Vector3{0,1,0}, obb.forward); },
				[&obb]() { return cross(Vector3{0,0,1}, obb.right); },
				[&obb]() { return cross(Vector3{0,0,1}, obb.up); },
				[&obb]() { return cross(Vector3{0,0,1}, obb.forward); }
			};

			for (auto& getAxis : axes) {
				auto axis = getAxis();
				//get the points that, after the projection, will be the outermost (both for the OBB and the AABB)
				auto obbExtremesIndexes = projectedObbExtremes(axis, obb);
				auto aabbExtremesIndexes = projectedAabbExtremes(axis);

				// |--------------------|MA     MB    overlap iff mB <= MA && MB >= mA (not overlap iff mB > MA || MB < mA)
				// mA             mB|-----------|
				if (dot(aabbVertices[aabbExtremesIndexes.first], axis) > dot(obbVertices[obbExtremesIndexes.second], axis) ||
					dot(aabbVertices[aabbExtremesIndexes.second], axis) < dot(obbVertices[obbExtremesIndexes.first], axis)) return false;
			}
			return true; //if we havent't found any axis where there is no overlap, boxes are colliding
		}

		/**
		 * @brief Implementation of the separating axis theorem between an /p Obb and an /p Aabb.
		 */
		static bool areColliding(const Obb& obb, const Aabb& aabb) {
			return areColliding(AabbForObb{ obb }, aabb);
		}

		/**
		 * @brief Checks whether 2 vectors are almost parallel.
		 */
		static bool almostParallel(const Vector3& lhs, const Vector3& rhs, float threshold = 0.01f) {
			return abs(glm::dot(lhs, rhs)) < threshold;
		}

		/**
		 * @brief Checks whether and OBB is "almost" an AABB.
		 */
		static bool almostAabb(const Obb& obb) {
			return 
				(almostParallel(obb.forward, Vector3{ 0,0,1 }) && almostParallel(obb.right, Vector3{ 1,0,0 })) ||
				(almostParallel(obb.forward, Vector3{ 0,0,1 }) && almostParallel(obb.up, Vector3{ 1,0,0 })) ||
				(almostParallel(obb.right, Vector3{ 0,0,1 }) && almostParallel(obb.forward, Vector3{ 1,0,0 })) ||
				(almostParallel(obb.right, Vector3{ 0,0,1 }) && almostParallel(obb.up, Vector3{ 1,0,0 })) ||
				(almostParallel(obb.up, Vector3{ 0,0,1 }) && almostParallel(obb.forward, Vector3{ 1,0,0 })) ||
				(almostParallel(obb.up, Vector3{ 0,0,1 }) && almostParallel(obb.right, Vector3{ 1,0,0 }));
		}

		/**
		 * @brief Given an axis, it returns the indexes of the vertices (min and max) that are most far apart (the extremes) on the projection of an AABB to the specified axis.
		 * The indexes refer to the order in which the function Aabb::getPoints return the vertices of an AABB.
		 * Look at the visualization in the project ProjectedAreaHeuristic-Visualization to better understand the logic behind this funciton.
		 *  
		 *  2_______________6
		 *  |\             .\
		 *  | \____________._\7
		 * 0| 3|          4. |
		 *  .\ |           . |
		 *  . \|1__________._|5
		 *  .  .           . .  
		 *  |--|-----------|-|    in this case A and D are the outermost., which correspond to points with index 0 (or 2) and 7 (or 5)
		 *  A  B           C D
		 * 
		 */
		static std::pair<int, int> projectedAabbExtremes(const Vector3& axis) {
			auto normalized = glm::normalize(axis);
			float x = normalized.x, y = normalized.y, z = normalized.z;

			if (x >= 0 && y >= 0 && z <= 0) return { 1, 6 }; //yellow
			if (x <= 0 && y <= 0 && z >= 0) return { 6, 1 }; //blue
			if (x >= 0 && y >= 0 && z >= 0) return { 0, 7 }; //white
			if (x <= 0 && y <= 0 && z <= 0) return { 7, 0 }; //black
			if (x >= 0 && y <= 0 && z <= 0) return { 3, 4 }; //red
			if (x <= 0 && y >= 0 && z >= 0) return { 4, 3 }; //cyan
			if (x >= 0 && y <= 0 && z >= 0) return { 2, 5 }; //magenta
			if (x <= 0 && y >= 0 && z <= 0) return { 5, 2 }; //green
		}

		/**
		 * @brief Given an axis and an OBB, it returns the indexes of the vertices (min and max) that are most far apart (the extremes) on the projection of an AABB to the specified axis.
		 * What happens is that the function transforms the axis into the OBB coordinate system, and then treats it like an AABB.
		 */
		static std::pair<int, int> projectedObbExtremes(const Vector3& axis, const Obb& obb) {
			Matrix3 newBasis{ obb.right, obb.up, obb.forward };
			Vector3 newAxis = glm::inverse(newBasis) * axis;
			return projectedAabbExtremes(newAxis);
		}
	}
}
