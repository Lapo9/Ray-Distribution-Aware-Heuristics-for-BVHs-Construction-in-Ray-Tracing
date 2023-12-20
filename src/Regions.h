#pragma once

#include <functional>

#include "Utilities.h"

namespace pah {
	class Region {
	public:
		/**
		 * @brief Returns whether the /p point is inside the region.
		 */
		virtual bool isInside(const Vector3& point) const = 0;
	};

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

		bool isInside(const Vector3& point) const override {
			return
				point.x >= min.x &&
				point.y >= min.y &&
				point.z >= min.z &&
				point.x <= max.x &&
				point.y <= max.y &&
				point.z <= max.z;
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
	};

	struct Obb : public Region {
		Vector3 center;
		Vector3 forward;
		Vector3 right;
		Vector3 up;
		Vector3 halfSize;

		Obb(const Vector3& center, const Vector3& halfSize, const Vector3& forward) : center{ center }, halfSize{ halfSize },
			forward{ glm::normalize(forward) }, right{ glm::cross(Vector3{0,1,0}, forward) }, up{ glm::cross(forward, right) } {
		}

		bool isInside(const Vector3& point) const override {
			using namespace glm;

			Vector3 d = point - center;
			//we project the distance vector between the point and the center in the reference system of the OBB, and we check the distances are < than the half size in all directions
			return
				abs(dot(d, right)) <= halfSize.x &&
				abs(dot(d, up)) <= halfSize.y &&
				abs(dot(d, forward)) <= halfSize.z;
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
						Vector3 worldVertex = Vector3{ glm::dot(obbVertex, right), glm::dot(obbVertex, up) , glm::dot(obbVertex, forward) } + center; //point in world space
						points[((i + 1) / 2) * 4 + ((j + 1) / 2) * 2 + ((k + 1) / 2) * 1] = worldVertex;
					}
			return points;
		}

		/**
		 * @brief Given an OBB it returns the tighest enclosing AABB.
		 */
		static Aabb enclosingAabb(const Obb& obb) {
			Aabb aabb{ Vector3{std::numeric_limits<float>::max()}, Vector3{ -std::numeric_limits<float>::max()} };
			auto vertices = obb.getPoints();

			//TODO probably there is a more efficient way
			for (auto& v : vertices) {
				//update max and min of enclosing AABB
				aabb.max = glm::max(aabb.max, v);
				aabb.min = glm::min(aabb.min, v);
			}
			return aabb;
		}
	};

	struct AabbForObb : public Region {
		Obb obb;
		Aabb aabb;

		AabbForObb(const Vector3& center, const Vector3& halfSize, const Vector3& forward) : obb{ center, halfSize, forward }, aabb{ Obb::enclosingAabb(obb) } {}

		bool isInside(const Vector3& point) const override {
			//check if the point is inside the AABB (cheap), if it is, check the OBB
			if (aabb.isInside(point)) return obb.isInside(point);
			return false;
		}
	};


	namespace collisionDetection {
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
		 * @brief Implementation of the separating axis theorem between an OBB and an AABB.
		 */
		static bool areColliding(const Obb& obb, const Aabb& aabb) {
			using namespace glm;

			//first, we check if the enclosing AABB of the OBB overlaps with the AABB (it can save a lot of time)
			bool aabbsColliding = areColliding(aabb, Obb::enclosingAabb(obb));
			if (aabbsColliding) return true;
			
			//then we check whether the OBB is "almost" an AABB (in this case we can approximate the collision to the AABB v AABB case)
			if (almostAabb(obb)) return aabbsColliding;

			//else, we have to use SAT
			auto obbVertices = obb.getPoints();
			auto abbVertices = aabb.getPoints();

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

			for (auto& axis : axes) {
				//TODO continue from here
			}
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
	}
}
