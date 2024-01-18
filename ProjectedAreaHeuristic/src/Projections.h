#pragma once

#include "Utilities.h"

/**
 * @brief Function and utilities to project points and @Aabb s to a plane.
 */
namespace pah::projection {

	/**
	 * @brief Returns the view matrix, to go from world space to camera space.
	 */
	static Matrix4 computeViewMatrix(const Vector3& eye, const Vector3& direction, const Vector3& up = Vector3{ 0.0f,1.0f,0.0f }) {
		return glm::lookAt(eye, eye + direction, up);
		//  right_x     right_y     right_z     -translation_x
		//  up_x        up_y        up_z        -translation_y
		//  forward_x   forward_y   forward_z   -translation_z
		//  0           0           0           1
		// Where:
		// forward = direction;
		// right = up X forward
		// up = forward X right
		// translation = (right . eye, up . eye, forward . eye) where A . B is dot(A, B)
	}

	/**
	 * @brief Returns a matrix to go from one space to another space with origin at @p origin and basis @p right, @p up and @p forward (as seen from the space we are currently in).
	 */
	static Matrix4 computeChangeOfCoordinatesMatrix(Vector3 right, Vector3 up, Vector3 forward, Vector3 origin) {
		using namespace glm;
		return Matrix4{
			Vector4{right.x, up.x, forward.x, 0},
			Vector4{right.y, up.y, forward.y, 0},
			Vector4{right.z, up.z, forward.z, 0},
			Vector4{-dot(right, origin), -dot(up, origin), -dot(forward, origin), 1}
		};
	}

	/**
	 * @brief Returns the perspective matrix given the frustum.
	 */
	static Matrix4 computePerspectiveMatrix(float f, float n, float halfWidth, float halfHeight) {
		float r = halfWidth, l = -halfWidth;
		float t = halfHeight, b = -halfHeight;

		return glm::frustum(l, r, b, t, n, f);

		//	n	0	0	0
		//	0	n	0	0
		//	0	0	f+n	-fn
		//	0	0	-1	0

		//this is a view-perspective matrix
		//  2n/(r-l)    0           -(r+l)/(r-l)		0
		//  0           2n/(t-b)    -(t+b)/(t-b)		0
		//  0           0           (f+n)/(f-n)		-2fn/(f-n)
		//  0           0           -1				0
	}

	/**
	 * @brief Returns the perspective matrix.
	 *
	 * @param fovs Horizontal and vertical FoVs in degrees.
	 */
	static Matrix4 computePerspectiveMatrix(float f, float n, std::tuple<float, float> fovs) {
		using namespace glm;
		auto [fovX, fovY] = fovs;

		//we use the catetus theorem: tan(a) = opposite / adjacent
		//                  y^ 
		//                  n|______ opp.
		//                   |     /.
		//                   |    / .
		//              adj. |   /  .
		//                   |  /   .
		//                   |a/    . 
		//    _ _ _ _ _ _ _ _|/_ _ _._ _ _ _ _ _ _>x 
		//                  O|      r

		float right = n * tan(radians(fovX) / 2.0f);
		float top = n * tan(radians(fovY) / 2.0f);
		float aspectRatio = right / top;

		return glm::perspective(radians(fovY), aspectRatio, n, f);
	}

	/**
	 * @brief Structure containing the parameters of a projection matrix.
	 * If the projection is not a perspective, @p fovX and @p fovY won't have any meaning.
	 */
	struct ProjectionMatrixParameters {
		float n, f, b, t, l, r, fovX, fovY, ratio;
	};

	/**
	 * @brief Given a perspective matrix, it returns its parameters (namely near, far, left, right, top and bottom planes, x and y field of views and the aspect ratio).
	 * 
	 * @param viewProjectionMatrix .
	 * @return .
	 */
	static ProjectionMatrixParameters extractPerspectiveMatrixParameters(const Matrix4 projectionMatrix) {
		const auto& M = projectionMatrix;

		float n = M[3][2] / (M[2][2] - 1.0f);
		float f = M[3][2] / (M[2][2] + 1.0f);
		float b = n * (M[2][1] - 1.0f) / M[1][1];
		float t = n * (M[2][1] + 1.0f) / M[1][1];
		float l = n * (M[2][0] - 1.0f) / M[0][0];
		float r = n * (M[2][0] + 1.0f) / M[0][0];
		float ratio = M[1][1] / M[0][0];

		//we use the catetus theorem: tan(a) = opposite / adjacent
		//                  y^ 
		//                  n|______ opp.
		//                   |     /.
		//                   |    / .
		//              adj. |   /  .
		//                   |  /   .
		//                   |a/    . 
		//    _ _ _ _ _ _ _ _|/_ _ _._ _ _ _ _ _ _>x 
		//  
		float fovY = glm::atan(t / n) * 2.0f;
		float fovX = glm::atan(r / n) * 2.0f;

		return ProjectionMatrixParameters{ n, f, b, t, l, r, fovX, fovY, ratio };

		//this is a view-perspective matrix
		//  2n/(r-l)    0           (r+l)/(r-l)		0				 <==>		1/ratio*tan(fovY/2)		0		0		0			[in this second representation the frustum is symmetric by definition (l = r and t = b)]
		//  0           2n/(t-b)    (t+b)/(t-b)		0				 <==>				0		1/tan(fovY/2)	0		0
		//  0           0           -(f+n)/(f-n)		-2fn/(f-n)			 <==>				0			0	-(f+n)/(f-n)	-2fn/(f-n)
		//  0           0           -1				0				 <==>				0			0		-1		0
	}


	/**
	 * @brief Function and utilities for orthographic projections.
	 */
	namespace orthographic {

		/**
		 * @brief Given a plane, projects the point to it.
		 */
		static Vector2 projectPoint(Vector4 point, Plane plane) {
			return computeViewMatrix(plane.getPoint(), plane.getNormal()) * point;
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
		static std::vector<Vector2> projectPoints(const std::vector<Vector3>& points, Plane plane) {
			std::vector<Vector2> projectedPoints;
			for (int i = 0; i < points.size(); ++i) {
				projectedPoints.emplace_back(projectPoint(points[i], plane));
			}
			return projectedPoints;
		}

		/**
		 * @brief Projects the @p Aabb to the specified plane.
		 */
		static std::array<Vector2, 8> projectAabb(const Aabb& aabb, Plane plane) {
			using namespace std;
			auto points = aabb.getPoints();
			auto projectedPoints = projectPoints(vector(points.begin(), points.end()), plane);
			array<Vector2, 8> result{};
			std::move(projectedPoints.begin(), projectedPoints.begin() + 8, result.begin());
			return result;
		}

		/**
		 * @brief Given an @p Aabb and a plane, it returns the area the @p Aabb projects on the plane.
		 */
		static float computeProjectedArea(const Aabb& aabb, Plane plane) {
			auto points = aabb.getPoints();
			std::vector<Vector3> keyPoints = { points[3], points[1], points[2], points[7] };
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


	/**
	 * @brief Functions and utilities for perspective projections.
	 */
	namespace perspective {
		/**
		 * @brief An HullInfo object keeps a list of the indexes of the vertices that are part of the convex hull.
		 * See hull table for more info.
		 */
		struct HullInfo {
			std::vector<int> vertices; //index of the indices creating the hull

			HullInfo() {}
			HullInfo(std::vector<int> vertices) : vertices{ vertices } {}

			/**
			 * @brief The indexing follows this scheme:
			 * |   5   |   4   |   3   |   2   |   1   |   0   |
			 * | back  | front |  top  | bott. | right | left  |
			 */
			static std::string getDescription(int i) {
				using namespace std;
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
		 * To get more info about how we index the vertices of an @p Aabb look at the function @p pah::Aabb::getPoints
		 */
		static std::array<HullInfo, 43> hullTable =
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


		static Vector2 projectPoint(Vector4 point, const Matrix4& viewProjectionMatrix) {
			return viewProjectionMatrix * point;
		}

		static Vector2 projectPoint(Vector3 point, const Matrix4& viewProjectionMatrix) {
			return projectPoint(Vector4{ point, 1.0f }, viewProjectionMatrix);
		}

		/**
		 * @brief Projects the given point based on the PoV using perspective.
		 * @param fovs Horizontal and vertical FoVs in degrees.
		 */
		static Vector2 projectPoint(Vector4 point, Pov pov, float far = 1000.0f, float near = 0.1f) {
			return projectPoint(point, computeViewMatrix(pov.position, pov.getDirection()) * computePerspectiveMatrix(far, near, { pov.fovX, pov.fovY }));
		}

		/**
		 * @brief Projects the given point based on the PoV using perspective. 1.0 is appended as w component of the point.
		 * @param fovs Horizontal and vertical FoVs in degrees.
		 */
		static Vector2 projectPoint(Vector3 point, Pov pov, float far = 1000.0f, float near = 0.1f) {
			return projectPoint(Vector4{ point, 1.0f }, pov, far, near);
		}

		static std::vector<Vector2> projectPoints(const std::vector<Vector3>& points, const Matrix4& viewProjectionMatrix) {
			std::vector<Vector2> projectedPoints;
			for (int i = 0; i < points.size(); ++i) {
				projectedPoints.emplace_back(projectPoint(points[i], viewProjectionMatrix));
			}
			return projectedPoints;
		}

		/**
		 * @brief Given some points it projects them based on the PoV, using perspective.
		 * @param fovs Horizontal and vertical FoVs in degrees.
		 */
		static std::vector<Vector2> projectPoints(const std::vector<Vector3>& points, Pov pov, float far = 1000.0f, float near = 0.1f) {
			Matrix4 viewProjectionMatrix = computeViewMatrix(pov.position, pov.getDirection()) * computePerspectiveMatrix(far, near, { pov.fovX, pov.fovY }); //we compute it here to avoid recomputing it for each point
			return projectPoints(points, viewProjectionMatrix);
		}

		/**
		 * @brief Given an @p Aabb it projects all of its points based on the PoV, using perspective.
		 * @param fovs Horizontal and vertical FoVs in degrees.
		 */
		static std::array<Vector2, 8> projectAabb(const Aabb& aabb, Pov pov, float far = 1000.0f, float near = 0.1f) {
			using namespace std;
			auto points = aabb.getPoints();
			auto projectedPoints = projectPoints(vector(points.begin(), points.end()), pov, far, near);
			array<Vector2, 8> result{};
			std::move(projectedPoints.begin(), projectedPoints.begin() + 8, result.begin());
			return result;
		}

		/**
		 * @brief Given the PoV and an @p Aabb it returns the corresponding index in the hull table, based on the relative position of the PoV and the @p Aabb.
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
		 * @brief Given an @p Aabb and the index of the hull table, it returns the 3D contour points (to be projected later on).
		 */
		static std::vector<Vector3> findContourPoints(const Aabb& aabb, int i) {
			using namespace std;
			const vector<int>& verticesIndexes = hullTable[i].vertices; //get the array of indexes of the contour vertices
			vector<Vector3> contourVertices(verticesIndexes.size());
			for (int j = 0; j < verticesIndexes.size(); j++) {
				contourVertices[j] = aabb.getPoint(verticesIndexes[j]);
			}
			return contourVertices;
		}

		/**
		 * @brief Given and @p Aabb and a PoV, it returns the 3D contour points by using the hull table and the relative position of the PoV and the @p Aabb.
		 */
		static std::vector<Vector3> findContourPoints(const Aabb& aabb, Vector3 pov) {
			return findContourPoints(aabb, findHullTableIndex(aabb, pov));
		}

		/**
		 * @brief Given the contour points of a convex 2D hull, it calculates its area.
		 * It uses a technique called contour integral.
		 */
		static float computeProjectedArea(const std::vector<Vector2>& contourPoints) {
			float area = 0.0f;
			//for each segment of the hull, we compute its SIGNED area w.r.t. the x-axis
			for (int i = 0; i < contourPoints.size(); ++i) {
				float width = contourPoints[i + 1 == contourPoints.size() ? 0 : i + 1].x - contourPoints[i].x; //width of the segment
				float meanHeight = (contourPoints[i + 1 == contourPoints.size() ? 0 : i + 1].y + contourPoints[i].y) / 2.0f; //mean height of the segment
				area += width * meanHeight;
			}
			assert((area <= 0, "Projected area <= 0"));
			return area;
		}

		/**
		 * @brief Given an @p Aabb and a PoV, computes the projected area of the @p Aabb.
		 * @param fovs Horizontal and vertical FoVs in degrees.
		 */
		static float computeProjectedArea(const Aabb& aabb, Pov pov, float far = 1000.0f, float near = 0.1f) {
			const auto& contourPoints = findContourPoints(aabb, pov.position);
			return contourPoints.size() == 0 ? 1000.0f : computeProjectedArea(projectPoints(contourPoints, pov, far, near));
		}
	}
}
