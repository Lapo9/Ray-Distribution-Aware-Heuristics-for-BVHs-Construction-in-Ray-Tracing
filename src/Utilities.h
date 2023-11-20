#pragma once

#include <glm/glm.hpp>
#include <vector>
#include <stdexcept>

namespace pah {
	using Vector3 = glm::vec3;
	using Vector2 = glm::vec2;
	using namespace std;
	using namespace utilities;


	struct Triangle {
		Vector3 v1, v2, v3;

		Vector3 center() {
			return (v1 + v2 + v3) / 3.0f;
		}
	};

	enum class Axis {
		X, Y, Z, None
	};

	struct Aabb {
		Vector3 min;
		Vector3 max;

		Aabb() {}

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

		static Aabb maxAabb() {
			Aabb res;
			res.min = Vector3{ numeric_limits<float>::min() };
			res.max = Vector3{ numeric_limits<float>::max() };
			return res;
		}
	};

	namespace utilities {
		static float at(const Vector3& vector, Axis axis) {
			if (axis == Axis::X) return vector.x;
			if (axis == Axis::Y) return vector.y;
			if (axis == Axis::Z) return vector.z;
			throw invalid_argument{ "Cannot use pah::Bvh::Axis::None as argument" };
		}

		static Axis third(Axis a1, Axis a2) {
			if (a1 == Axis::X && a2 == Axis::Y || a1 == Axis::Y && a2 == Axis::X) return Axis::Z;
			if (a1 == Axis::X && a2 == Axis::Z || a1 == Axis::Z && a2 == Axis::X) return Axis::Y;
			if (a1 == Axis::Y && a2 == Axis::Z || a1 == Axis::Z && a2 == Axis::Y) return Axis::X;
			throw invalid_argument{ "Cannot use pah::Axis::None as argument" };
		}

		static tuple<Axis, Axis> other2(Axis axis) {
			if (axis == Axis::X) return { Axis::Y, Axis::Z };
			if (axis == Axis::Y) return { Axis::X, Axis::Z };
			if (axis == Axis::Z) return { Axis::X, Axis::Y };
			throw invalid_argument{ "Cannot use pah::Bvh::Axis::None as argument" };
		}
	}
}
