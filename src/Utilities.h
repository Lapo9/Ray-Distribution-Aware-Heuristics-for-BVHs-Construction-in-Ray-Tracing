#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <vector>
#include <array>
#include <random>
#include <algorithm>
#include <limits>
#include <iostream>
#include <stdexcept>

#include  "../libs/json.hpp"

namespace pah {
	using Vector2 = glm::vec2;
	using Vector3 = glm::vec3;
	using Vector4 = glm::vec4;
	using Matrix4 = glm::mat4;
	using namespace std;

	template<typename D>
	concept Distribution3d = requires(D distr, mt19937 rng) {
		{ distr(rng) } -> std::same_as<Vector3>;
	};


	class Uniform3dDistribution {
	public:
		Uniform3dDistribution(float minX, float maxX, float minY, float maxY, float minZ, float maxZ) 
			: distributionX{ minX, maxX }, distributionY{ minY, maxY }, distributionZ{ minZ, maxZ } {}

		Vector3 operator()(mt19937& rng) {
			return Vector3{ distributionX(rng), distributionY(rng), distributionZ(rng) };
		}

	private:
		uniform_real_distribution<> distributionX;
		uniform_real_distribution<> distributionY;
		uniform_real_distribution<> distributionZ;
	};


	struct Triangle {
		Vector3 v1, v2, v3;

		Triangle() = delete;
		Triangle(Vector3 v1, Vector3 v2, Vector3 v3) : v1{ v1 }, v2{ v2 }, v3{ v3 } {}
		Triangle(const Triangle&) = default;
		Triangle& operator=(const Triangle&) = default;
		Triangle(Triangle&&) = default;
		Triangle& operator=(Triangle&&) = default;
		~Triangle() = default;

		Vector3 center() const {
			return (v1 + v2 + v3) / 3.0f;
		}

		template<Distribution3d D, Distribution3d D2>  
		static Triangle random(mt19937& rng, D& firstVertexDistribution, D2& otherVerticesDistributions) {
			Vector3 v1 = firstVertexDistribution(rng);
			Vector3 v2 = v1 + otherVerticesDistributions(rng);
			Vector3 v3 = v1 + otherVerticesDistributions(rng);
			return Triangle{ v1, v2, v3 };
		}

		template<Distribution3d D, Distribution3d D2>
		static vector<Triangle> generateRandom(int qty, mt19937& rng, D& firstVertexDistribution, D2& otherVerticesDistributions) {
			vector<Triangle> triangles;
			for (int i = 0; i < qty; ++i) {
				triangles.emplace_back(random(rng, firstVertexDistribution, otherVerticesDistributions));
			}
			return triangles;
		}
	};

	enum class Axis {
		X, Y, Z, None
	};

	struct Aabb {
		Vector3 min;
		Vector3 max;

		Aabb() = default;

		/**
		 * @brief Creates the tightest possible axis aligned bounding box for the given list of triangles.
		 */
		Aabb(const vector<const Triangle*>& triangles) : min{ numeric_limits<float>::max() }, max{ -numeric_limits<float>::max() } {
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

		Aabb(const Aabb&) = default;
		Aabb& operator=(const Aabb&) = default;
		Aabb(Aabb&&) = default;
		Aabb& operator=(Aabb&&) = default;
		~Aabb() = default;

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
		 * See pah::Aabb::getPoint for more info about the logic of this layout.
		 */
		array<Vector3, 8> getPoints() const {
			array<Vector3, 8> points{};
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

		static Aabb maxAabb() {
			Vector3 min{ -numeric_limits<float>::max() };
			Vector3 max{ numeric_limits<float>::max() };
			return Aabb{ min, max };
		}
	};

	struct Plane {
	private: 
		Vector3 normal;
		Vector3 point;

	public:
		Plane() : point{}, normal{ 0,0,1 } {}
		Plane(Vector3 point, Vector3 normal) : point{ point }, normal{ glm::normalize(normal) } {}

		const Vector3& getPoint() const { return point; }
		void setPoint(Vector3 point) { this->point = point; }
		const Vector3& getNormal() const { return normal; }
		void setNormal(Vector3 normal) { this->normal = glm::normalize(normal); }
	};

	struct Pov {
	private:
		Vector3 position;
		Vector3 direction;
		Vector3 up;

	public:
		Pov() : position{}, direction{ Vector3{1, 0, 0} }, up{ 0,1,0 } {}
		Pov(Vector3 position, Vector3 direction, Vector3 up = Vector3{ 0.0f, 1.0f, 0.0f }) : position{ position }, direction{ direction }, up{ up } {}
		Pov(Plane plane, Vector3 up = Vector3{ 0.0f, 1.0f, 0.0f }) : position{ plane.getPoint()}, direction{plane.getNormal()}, up{up} {}

		const Vector3& getPosition() const { return position; }
		void setPosition(Vector3 position) { this->position = position; }
		const Vector3& getDirection() const { return direction; }
		void setDirection(Vector3 direction) { this->direction = glm::normalize(direction); }		
		const Vector3& getUp() const { return up; }
		void setUp(Vector3 up) { this->up = glm::normalize(up); }
	};


	namespace utilities {

		/**
		 * @brief Given a vector and an axis, returns the corresponding component of the vector.
		 */
		static float at(const Vector3& vector, Axis axis) {
			if (axis == Axis::X) return vector.x;
			if (axis == Axis::Y) return vector.y;
			if (axis == Axis::Z) return vector.z;
			throw invalid_argument{ "Cannot use pah::Axis::None as argument" };
		}

		/**
		 * @brief Given 2 axis, it returns the remaining one.
		 */
		static Axis third(Axis a1, Axis a2) {
			if (a1 == Axis::X && a2 == Axis::Y || a1 == Axis::Y && a2 == Axis::X) return Axis::Z;
			if (a1 == Axis::X && a2 == Axis::Z || a1 == Axis::Z && a2 == Axis::X) return Axis::Y;
			if (a1 == Axis::Y && a2 == Axis::Z || a1 == Axis::Z && a2 == Axis::Y) return Axis::X;
			throw invalid_argument{ "Cannot use pah::Axis::None as argument" };
		}

		/**
		 * @brief Given an axis it returns the remaining 2.
		 */
		static tuple<Axis, Axis> other2(Axis axis) {
			if (axis == Axis::X) return { Axis::Y, Axis::Z };
			if (axis == Axis::Y) return { Axis::X, Axis::Z };
			if (axis == Axis::Z) return { Axis::X, Axis::Y };
			throw invalid_argument{ "Cannot use pah::Axis::None as argument" };
		}


		class TimeLogger {
		public:
			template<same_as<chrono::duration<long long, std::nano>>... FinalActions>
			TimeLogger(function<void(FinalActions... duration)>& finalActions) {
				(this->finalActions.push_back(finalActions), ...);
			}
			
			TimeLogger(const TimeLogger&) = default;
			TimeLogger& operator=(const TimeLogger&) = default;
			TimeLogger(TimeLogger&&) = default;
			TimeLogger& operator=(TimeLogger&&) = default;
			
			~TimeLogger() {
				if(!alreadyStopped) stop();
			}

			void stop() {
				auto duration = chrono::high_resolution_clock::now() - startTime;
				for (const auto& finalAction : finalActions) {
					finalAction(duration);
				}
			}

			//possible final actions

			static void print(chrono::duration<long long, std::nano> duration, string name) {
				std::cout << name << " " << duration << std::endl;
			}

			static void json(chrono::duration<long long, std::nano> duration, nlohmann::json& jsonOut, string label) {
				jsonOut[label] = duration.count();
			}

		private:
			chrono::time_point<chrono::high_resolution_clock> startTime;
			vector<function<void(chrono::duration<long long, std::nano> duration)>> finalActions;
			bool alreadyStopped = false;
		};
	}


	namespace projection {

		/**
		 * @brief Returns the view matrix, to go from world space to camera space.
		 */
		static Matrix4 computeViewMatrix(Pov pov) {
			return glm::lookAt(pov.getPosition(), pov.getPosition() + pov.getDirection(), pov.getUp());
			//  right_x     right_y     right_z     -translation_x
			//  up_x        up_y        up_z        -translation_y
			//  forward_x   forward_y   forward_z   -translation_z
			//  0           0           0           1
			// Where:
			// forward = pov.Direction;
			// right = pov.Up X forward
			// up = forward X right
		}

		/**
		 * @brief Returns the perspective matrix given the frustum.
		 */
		static Matrix4 computePerspectiveMatrix(float f, float n, float halfWidth, float halfHeight) {
			float r = halfWidth, l = -halfWidth;
			float t = halfHeight, b = -halfHeight;
			float aspectRatio = (r - l) / (t - b);
			Vector2 topNearPoint = glm::normalize(Vector2{ n, t });
			float fovY = glm::atan(topNearPoint.x, topNearPoint.y) * 2.0f;

			return glm::perspective(glm::degrees(fovY), aspectRatio, n, f);

			//  2n/(r-l)    0           (l-r)/(r-l)     0
			//  0           2n/(t-b)    (b-t)/(t-b)     0
			//  0           0           (f+n)/(f-n)     -2fn/(f-n)
			//  0           0           1               0
		}

		/**
		 * @brief Returns the perspective matrix.
		 * 
		 * @param fovs Horizontal and vertical FoVs in degrees.
		 */
		static Matrix4 computePerspectiveMatrix(float f, float n, tuple<float, float> fovs) {
			auto [fovX, fovY] = fovs;
			float right = glm::sin(glm::radians(fovX) / 2.0f);
			float top = glm::sin(glm::radians(fovY) / 2.0f);
			float aspectRatio = right / top;
			
			return glm::perspective(fovY, aspectRatio, n, f);
		}


		namespace orthographic {

			/**
			 * @brief Given a plane, projects the point to it.
			 */
			static Vector2 projectPoint(Vector4 point, Plane plane) {
				return computeViewMatrix(Pov{ plane }) * point;
			}

			/**
			 * @brief Given a plane, projects the point to it. 1.0 is appended as w component of the point.
			 */
			static Vector2 projectPoint(Vector3 point, Plane plane) {
				return projectPoint(Vector4{ point, 1.0f }, plane);
			}

			/**
			 * @brief Projects the given points to the specified plane.
			 */
			static vector<Vector2> projectPoints(const vector<Vector3>& points, Plane plane) {
				vector<Vector2> projectedPoints;
				for (int i = 0; i < points.size(); ++i) {
					projectedPoints.emplace_back(projectPoint(points[i], plane));
				}
				return projectedPoints;
			}

			/**
			 * @brief Projects the AABB to the specified plane.
			 */
			static array<Vector2, 8> projectAabb(const Aabb& aabb, Plane plane) {
				auto points = aabb.getPoints();
				auto projectedPoints = projectPoints(vector(points.begin(), points.end()), plane);
				array<Vector2, 8> result{};
				std::move(projectedPoints.begin(), projectedPoints.begin() + 8, result.begin());
				return result;
			}

			/**
			 * @brief Given an AABB and a plane, it returns the area the AABB projects on the plane.
			 */
			static float computeProjectedArea(const Aabb& aabb, Plane plane) {
				auto points = aabb.getPoints();
				vector<Vector3> keyPoints = { points[3], points[1], points[2], points[7] };
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


		namespace perspective {
			/**
			 * @brief An HullInfo object keeps a list of the indexes of the vertices that are part of the convex hull.
			 * See hull table for more info.
			 */
			struct HullInfo {
				vector<int> vertices; //index of the indices creating the hull

				HullInfo() {}
				HullInfo(vector<int> vertices) : vertices{ vertices } {}

				/**
				 * @brief The indexing follows this scheme:
				 * |   5   |   4   |   3   |   2   |   1   |   0   |
				 * | back  | front |  top  | bott. | right | left  |
				 */
				static string getDescription(int i) {
					string description = "";
					array<string, 6> keywords = { "left", "right", "bottom", "top", "front", "back" };
					for (int j = 0; j < 6; j++) {
						if ((i & 1) != 0) description += keywords[j] + " ";
						i >>= 1;
					}
					return description;
				}
			};

			/** @brief This array contains arrays containing the indices of the veritces making up the hull.
			 * The indexing of this array follows this rule:
			 * |   5   |   4   |   3   |   2   |   1   |   0   |
			 * | back  | front |  top  | bott. | right | left  |
			 * So, for example index 9 = 001001b will contain the indexes when the AABB is seen from top-left, whereas index 4 = 000100b will contain the indexes when the AABB is seen from bottom.
			 * Of course some indexes will be empty, since you cannot see the AABB from bottom and top at the same time, therefore xx11xxb = 13 is empty.
			 * To get more info about how we index the vertices of an AABB look at the function pah::Aabb::getPoints
			 */
			static array<HullInfo, 43> hullTable = 
			{
				HullInfo{},
				HullInfo{{ 1, 0, 2, 3 }},
				HullInfo{{ 5, 7, 6, 4 }},
				HullInfo{},
				HullInfo{{ 1, 5, 4, 0 }},
				HullInfo{{ 1, 5, 4, 0, 2, 3 }},
				HullInfo{{ 1, 5, 7, 6, 4, 0 }},
				HullInfo{},
				HullInfo{{ 7, 3, 2, 6 }},
				HullInfo{{ 0, 2, 6, 7, 3, 1 }},
				HullInfo{{ 7, 3, 2, 6, 4, 5 }},
				HullInfo{},
				HullInfo{},
				HullInfo{},
				HullInfo{},
				HullInfo{},
				HullInfo{{ 1, 3, 7, 5 }},
				HullInfo{{ 1, 0, 2, 3, 7, 5 }},
				HullInfo{{ 1, 3, 7, 6, 4, 5 }},
				HullInfo{},
				HullInfo{{ 1, 3, 7, 5, 4, 0 }},
				HullInfo{{ 7, 5, 4, 0, 2, 3 }},
				HullInfo{{ 1, 3, 7, 6, 4, 0 }},
				HullInfo{},
				HullInfo{{ 1, 3, 2, 6, 7, 5 }},
				HullInfo{{ 1, 0, 2, 6, 7, 5 }},
				HullInfo{{ 1, 3, 2, 6, 4, 5 }},
				HullInfo{},
				HullInfo{},
				HullInfo{},
				HullInfo{},
				HullInfo{},
				HullInfo{{ 0, 4, 6, 2 }},
				HullInfo{{ 0, 4, 6, 2, 3, 1 }},
				HullInfo{{ 5, 7, 6, 2, 0, 4 }},
				HullInfo{},
				HullInfo{{ 1, 5, 4, 6, 2, 0 }},
				HullInfo{{ 1, 5, 4, 6, 2, 3 }},
				HullInfo{{ 1, 5, 7, 6, 2, 0 }},
				HullInfo{},
				HullInfo{{ 7, 3, 2, 0, 4, 6 }},
				HullInfo{{ 1, 0, 4, 6, 7, 3 }},
				HullInfo{{ 5, 7, 3, 2, 0, 4 }}
			};


			/**
			 * @brief Projects the given point based on the PoV using perspective.
			 * @param fovs Horizontal and vertical FoVs in degrees.
			 */
			static Vector2 projectPoint(Vector4 point, Pov pov, tuple<float, float> fovs = tuple{ 90.0f, 90.0f }, float far = 1000.0f, float near = 0.1f) {
				return computeViewMatrix(Pov{ pov }) * computePerspectiveMatrix(far, near, fovs) * point;
			}

			/**
			 * @brief Projects the given point based on the PoV using perspective. 1.0 is appended as w component of the point.
			 * @param fovs Horizontal and vertical FoVs in degrees.
			 */
			static Vector2 projectPoint(Vector3 point, Pov pov, tuple<float, float> fovs = tuple{ 90.0f, 90.0f }, float far = 1000.0f, float near = 0.1f) {
				return projectPoint(Vector4{ point, 1.0f }, pov, fovs, far, near);
			}

			/**
			 * @brief Given some points it projects them based on the PoV, using perspective.
			 * @param fovs Horizontal and vertical FoVs in degrees.
			 */
			static vector<Vector2> projectPoints(const vector<Vector3>& points, Pov pov, tuple<float, float> fovs = tuple{ 90.0f, 90.0f }, float far = 1000.0f, float near = 0.1f) {
				vector<Vector2> projectedPoints;
				for (int i = 0; i < points.size(); ++i) {
					projectedPoints.emplace_back(projectPoint(points[i], pov, fovs, far, near));
				}
				return projectedPoints;
			}

			/**
			 * @brief Given an AABB it projects all of its points based on the PoV, using perspective.
			 * @param fovs Horizontal and vertical FoVs in degrees.
			 */
			static array<Vector2, 8> projectAabb(const Aabb& aabb, Pov pov, tuple<float, float> fovs = tuple{ 90.0f, 90.0f }, float far = 1000.0f, float near = 0.1f) {
				auto points = aabb.getPoints();
				auto projectedPoints = projectPoints(vector(points.begin(), points.end()), pov, fovs, far, near);
				array<Vector2, 8> result{};
				std::move(projectedPoints.begin(), projectedPoints.begin() + 8, result.begin());
				return result;
			}

			/**
			 * @brief Given the PoV and an AABB it returns the corresponding index in the hull table, based on the relative position of the PoV and the AABB..
			 */
			static int findHullTableIndex(const Aabb& aabb, Vector3 pov) {
				int i = 0;
				if (pov.x < aabb.min.x) i |= 1;
				else if (pov.x > aabb.max.x) i |= 2;
				if (pov.y < aabb.min.y) i |= 4;
				else if (pov.y > aabb.max.y) i |= 8;
				if (pov.z < aabb.min.z) i |= 32;
				else if (pov.z > aabb.max.z) i |= 16;
				return i;
			}

			/**
			 * @brief Given an AABB and the index of the hull table, it returns the 3D contour points (to be projected later on).
			 */
			static vector<Vector3> findContourPoints(const Aabb& aabb, int i) {
				const vector<int>& verticesIndexes = hullTable[i].vertices; //get the array of indexes of the contour vertices
				vector<Vector3> contourVertices;
				for (int j = 0; j < verticesIndexes.size(); j++) {
					contourVertices[j] = aabb.getPoint(verticesIndexes[j]);
				}
				return contourVertices;
			}

			/**
			 * @brief Given and AABB and a PoV, it returns the 3D contour points by using the hull table and the relative position of the PoV and the AABB.
			 */
			static vector<Vector3> findContourPoints(const Aabb& aabb, Vector3 pov) {
				return findContourPoints(aabb, findHullTableIndex(aabb, pov));
			}

			/**
			 * @brief Given the contour points of a convex 2D hull, it calculates its area.
			 * It uses a technique called contour integral.
			 */
			static float computeProjectedArea(const vector<Vector2>& contourPoints) {
				float area = 0.0f;
				//for each segment of the hull, we compute its SIGNED area w.r.t. the x-axis
				for (int i = 0; i < contourPoints.size(); ++i) {
					float width = contourPoints[i + 1 == contourPoints.size() ? 0 : i + 1].x - contourPoints[i].x; //width of the segment
					float meanHeight = (contourPoints[i + 1 == contourPoints.size() ? 0 : i + 1].y + contourPoints[i].y) / 2.0f; //mean height of the segment
					area += width * meanHeight;
				}
				assert(( - area <= 0, "Projected area <= 0"));
				return -area;
			}

			/**
			 * @brief Given an AABB and a PoV, computes the projected area of the AABB.
			 * @param fovs Horizontal and vertical FoVs in degrees.
			 */
			static float computeProjectedArea(const Aabb& aabb, Pov pov, tuple<float, float> fovs = tuple{ 90.0f, 90.0f }, float far = 1000.0f, float near = 0.1f) {
				return computeProjectedArea(projectPoints(findContourPoints(aabb, pov.getPosition()), pov, fovs, far, near));
			}
		}
	}
}
