#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <vector>
#include <array>
#include <stdexcept>

namespace pah {
	using Vector2 = glm::vec2;
	using Vector3 = glm::vec3;
	using Vector4 = glm::vec4;
	using Matrix4 = glm::mat4;
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

		/**
		 * @brief Given an AABB (min-max form) returns the array of it 8 vertices with this layout:
		 *  2_______________6  
		 *  |\              \ 
		 *  | \______________\7
		 * 0| 3|          4. |
		 *   \ |             |
		 *    \|_____________|
		 *     1             5
		 */
		array<Vector3, 8> getPoints() const {
			array<Vector3, 8> points;
			for (int i = 0; i < 2; ++i)
				for (int j = 0; j < 2; ++j)
					for (int k = 0; k < 2; ++k) {
						Vector3 vertex{ i, j, k }; //selects which vertex we are considering (e.g. top-left-back)
						points[i * 4 + j * 2 + k] = min + vertex * size();
					}
		}

		static Aabb maxAabb() {
			Aabb res;
			res.min = Vector3{ numeric_limits<float>::min() };
			res.max = Vector3{ numeric_limits<float>::max() };
			return res;
		}
	};

	struct Plane {
		Vector3 point;
		Vector3 normal;
	};

	struct Pov {
		Vector3 position;
		Vector3 direction;
		Vector3 up;

		Pov(Vector3 position, Vector3 direction, Vector3 up = Vector3{ 0.0f, 1.0f, 0.0f }) : position{ position }, direction{ direction }, up{ up } {}
		Pov(Plane plane, Vector3 up = Vector3{ 0.0f, 1.0f, 0.0f }) : position{ plane.point }, direction{ plane.normal }, up{ up } {}
	};

	namespace utilities {
		static float at(const Vector3& vector, Axis axis) {
			if (axis == Axis::X) return vector.x;
			if (axis == Axis::Y) return vector.y;
			if (axis == Axis::Z) return vector.z;
			throw invalid_argument{ "Cannot use pah::Axis::None as argument" };
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
			throw invalid_argument{ "Cannot use pah::Axis::None as argument" };
		}
	}


	namespace projection {
		static Matrix4 computeViewMatrix(Pov pov) {
			return glm::lookAt(pov.position, pov.position + pov.direction, pov.up);
			//  right_x     right_y     right_z     -translation_x
			//  up_x        up_y        up_z        -translation_y
			//  forward_x   forward_y   forward_z   -translation_z
			//  0           0           0           1
			// Where:
			// forward = pov.Direction;
			// right = pov.Up X forward
			// up = forward X right
		}

		static Matrix4 computePerspectiveMatrix(float f, float n, float halfWidth, float halfHeight) {
			float r = halfWidth, l = -halfWidth;
			float t = halfHeight, b = -halfHeight;
			float aspectRatio = (r - l) / (t - b);
			Vector2 topNearPoint = glm::normalize(Vector2{ n, t });
			float fovY = glm::atan(topNearPoint.x, topNearPoint.y) * 2.0f;

			return glm::perspective(fovY, aspectRatio, n, f);

			//  2n/(r-l)    0           (l-r)/(r-l)     0
			//  0           2n/(t-b)    (b-t)/(t-b)     0
			//  0           0           (f+n)/(f-n)     -2fn/(f-n)
			//  0           0           1               0
		}

		static Matrix4 computePerspectiveMatrix(float f, float n, tuple<float, float> fov) {
			auto [fovX, fovY] = fov;
			float right = glm::sin(fovX / 2.0f);
			float top = glm::sin(fovY / 2.0f);
			float aspectRatio = right / top;
			
			return glm::perspective(fovY, aspectRatio, n, f);
		}

		namespace orthographic {
			static Vector2 projectPoint(Vector4 point, Plane plane) {
				return computeViewMatrix(Pov{ plane }) * point;
			}

			static Vector2 projectPoint(Vector3 point, Plane plane) {
				return computeViewMatrix(plane) * Vector4 { point, 1.0f };
			}

			static array<Vector2, 8> projectAabb(const Aabb& aabb, Plane plane) {
				auto points = aabb.getPoints();
				return projectPoints(points, plane);
			}

			template<int N>
			static array<Vector2, N> projectPoints(const array<Vector3, N>& points, Plane plane) {
				Matrix4 viewMatrix = computeViewMatrix(plane);
				array<Vector2, N> projectedPoints;
				for (int i = 0; i < 8; ++i) {
					projectedPoints[i] = viewMatrix * Vector4{ points[i], 1.0f };
				}
				return projectedPoints;
			}

			static float computeProjectedArea(const Aabb& aabb, Plane plane) {
				auto points = aabb.getPoints();
				array<Vector3, 4> keyPoints = { points[3], points[1], points[2], points[7] };
				auto projectedPoints = projectPoints(keyPoints, plane);

				Vector3 side1 = Vector3{ projectedPoints[0] - projectedPoints[1], 0.0f };
				Vector3 side2 = Vector3{ projectedPoints[0] - projectedPoints[2], 0.0f };
				Vector3 side3 = Vector3{ projectedPoints[0] - projectedPoints[3], 0.0f };

				//  2_______________                                                                                           _______________
				//  |\              \                                                                                         |               \
				//  | \______________\7                                                                                       |                \
				//  | 3|             |   ==> We have 3 parallelograms, the sum of their areas is the area of the polygon ==>  |                |
				//   \ |             |                                                                                         \               |
				//    \|_____________|                                                                                          \______________|
				//     1

				float area1 = glm::abs(glm::length(glm::cross(side1, side2)));
				float area2 = glm::abs(glm::length(glm::cross(side2, side3)));
				float area3 = glm::abs(glm::length(glm::cross(side1, side3)));

				return area1 + area2 + area3;
			}
		}


	}
}
