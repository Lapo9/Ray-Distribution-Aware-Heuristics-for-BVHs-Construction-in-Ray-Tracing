#pragma once

#include <functional>
#include <limits>
#include "glm/glm.hpp"

#include "Utilities.h"
#include "settings.h"

namespace pah {
	//forward declarations
	class Region;
	struct Aabb;
	struct Obb;
	struct AabbForObb;
	struct Frustum;
	namespace collisionDetection {
		static bool areColliding(const Aabb& aabb1, const Aabb& aabb2);
		static bool areColliding(const AabbForObb& aabbForObb, const Aabb& aabb);
		static bool areColliding(const Obb& obb, const Aabb& aabb);
		static bool areColliding(const Frustum& obb, const Aabb& aabb);
		static bool almostParallel(const Vector3& lhs, const Vector3& rhs, float threshold);
		static bool almostAabb(const Obb& obb);
		static std::pair<int, int> projectedAabbExtremes(const Vector3& axis);
		static std::pair<int, int> projectedObbExtremes(const Vector3& axis, const Obb& obb);
	}
	namespace projection {
		struct ProjectionMatrixParameters;
	}

	/**
	 * @brief Base class for anything that can represent a 3D space region.
	 */
	class Region {
	public:
		/**
		 * @brief Returns whether the @p point is inside the region.
		 */
		virtual bool contains(const Vector3& point) const = 0;

		/**
		 * @brief Returns the enclosing @p Aabb.
		 */
		virtual Aabb enclosingAabb() const = 0;

		/**
		 * @brief Returns whether there is a collision between this @p Region and an @p Aabb.
		 * We make this a method in order to make use of the runtime polymorphism features of C++.
		 */
		virtual bool isCollidingWith(const Aabb& aabb) const = 0;

		/**
		 * @brief Returns whether this @p Region fully contains the specified @p Aabb.
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
		Aabb(const std::vector<const Triangle*>& triangles);
		Aabb(const Vector3& min, const Vector3& max);

		bool contains(const Vector3& point) const override;

		Aabb enclosingAabb() const override;
		
		bool isCollidingWith(const Aabb& aabb) const override;

		bool fullyContains(const Aabb& aabb) const override;

		/**
		 * @brief Returns the center of the AABB.
		 */
		Vector3 center() const;

		/**
		 * @brief Returns the dimension of this AABB.
		 */
		Vector3 size() const;

		/**
		 * @brief Returns the surface area of the AABB.
		 */
		float surfaceArea() const;

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
		std::array<Vector3, 8> getPoints() const;

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
		Vector3 getPoint(int i) const;

		/**
		 * @brief Returns the biggest AABB possible.
		 */
		static Aabb maxAabb();

		/**
		 * @brief Returns the biggest negative AABB possible.
		 */
		static Aabb minAabb();

		/**
		 * @brief Adds toghether 2 @p Aabb s.
		 */
		Aabb& operator+=(const Aabb& lhs);

		/**
		 * @brief Adds toghether 2 @p Aabb s.
		 */
		friend Aabb& operator+(Aabb lhs, const Aabb& rhs);
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

		Obb(const Vector3& center, const Vector3& halfSize, const Vector3& forward);

		bool contains(const Vector3& point) const override;

		Aabb enclosingAabb() const override;

		bool isCollidingWith(const Aabb& aabb) const override;

		bool fullyContains(const Aabb& aabb) const override;


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
		std::array<Vector3, 8> getPoints() const;
	};

	/**
	 * @brief An OBB with an enclosing AABB.
	 */
	struct AabbForObb : public Region {
		Obb obb;
		Aabb aabb;

		AabbForObb(const Vector3& center, const Vector3& halfSize, const Vector3& forward);
		AabbForObb(const Obb& obb);

		bool contains(const Vector3& point) const override;

		Aabb enclosingAabb() const override;

		bool isCollidingWith(const Aabb& aabb) const override;

		bool fullyContains(const Aabb& aabb) const override;
	};

	/**
	 * @brief Describes a 3D rectangular frustum.
	 */
	struct Frustum : public Region {
		Frustum(const Matrix4& viewProjectionMatrix);

		/**
		 * @brief Builds a frustum given the forward direction, the view point, fan and near planes and the field of views.
		 */
		Frustum(const Pov& pov, float far, float near, float fovX, float fovY);

		// TODO finish
		/* Frustum(const Vector3& lbf, const Vector3& rbf, const Vector3& ltf, const Vector3& lbb) {
			using namespace glm;

			Vector3 right = rbf - lbf;
			Vector3 up = ltf - lbf;
			assert(abs(dot(right, up)) < TOLERANCE, "lbf, rbf and ltf should be vertices of a rectangle, but the segments lbf-rbf and lbf-ltf are not perpendicular.");
			
			Vector3 forward = normalize(cross(right, up));
			Vector3 center = (rbf + ltf) / 2.0f; //center of the front face of the frustum

			Vector3 oblique = lbb - lbf;
			
			//to find the Pov we must find the intersection between oblique and forward
			//if the intersection is not present, it means that the 4 points cannot create a frustum (we can do nothing)

			//when we find the Pov, we can build the view matrix.
			//after this, we transform lbf and lbb to view space: lbf.x is left, lbf.y is bottom, lbf.z is near, lbb.z is far.
			//with this we can now build the perspective matrix, and the view-perspective matrix.

		}*/

		bool contains(const Vector3& point) const override;

		Aabb enclosingAabb() const override;

		bool isCollidingWith(const Aabb& aabb) const override;

		bool fullyContains(const Aabb& aabb) const override;

		std::array<Vector3, 8> getPoints() const;

		std::array<Vector3, 6> getEdgesDirections() const;

		std::array<Vector3, 6> getFacesNormals() const;

		/**
		 * @brief Returns the stored view projection matrix..
		 */
		Matrix4 getViewProjectionMatrix() const;

		/**
		 * @brief Extracts the view matrix from the view projection matrix, by using the @p Frustum coordinate system (right, up, forward).
		 * Since the view matrix is computed starting from the view projection matrix, it can have some rounding errors caused by floating points.
		 */
		Matrix4 extractViewMatrix() const;

		/**
		 * @brief Extracts the perspective matrix from the view projection matrix and the extracted view matrix. VPm = Pm * Vm ==> Pm = VPm * Vm^(-1).
		 * Since the perspective matrix is computed starting from the view projection matrix, it can have some rounding errors caused by floating points.
		 */
		Matrix4 extractProjectionMatrix() const;

		const Vector3& getRight() const;
		const Vector3& getUp() const;
		const Vector3& getForward() const;
		projection::ProjectionMatrixParameters getViewProjectionMatrixParameters() const;

	private:
		/**
		 * @brief Fills the array containing the normals to the 6 faces of the frustum.
		 */
		void fillFacesNormals();

		/**
		 * @brief Fills the array containing the 6 directions of the 12 edges of a frustum.
		 * There are only 6 because the "right" and "top" directions are repeated on 4 edges, whereas the "forward" edges are oblique.
		 * @pre This function should only be called after @p fillVertices.
		 */
		void fillEdgesDirection();
		
		/**
		 * @brief Fills the array containing the 8 vertices of the frustum.
		 */
		void fillVertices();

		/**
		 * @brief Calculates the @p Aabb that more tightly encloses this @p Frustum.
		 * @pre This function should only be called after @p fillVertices.
		 */
		void fillEnclosingAabb();

		Matrix4 viewProjectionMatrix; //a frustum can be represented as a projection matrix (to define its shape) and a view matrix (to set its position)
		std::array<Vector3, 6> facesNormals; //useful for the SAT algorithm for collision detection
		std::array<Vector3, 6> edgesDirections; //useful for the SAT algorithm for collision detection
		std::array<Vector3, 8> vertices; //useful for the SAT algorithm for collision detection
		Aabb enclosingAabbObj; //the smallest Aabb that encloses the Frustum
	};

	/**
	 * @brief Functions and utilities to detect whether there is a collision between 2 @p Region s.
	 */
	namespace collisionDetection {

		/**
		 * @brief Returns whether 2 @p Aabb s are colliding.
		 */
		static bool areColliding(const Aabb& aabb1, const Aabb& aabb2);

		/**
		 * @brief Implementation of the separating axis theorem between an @p AabbForObb and an @p Aabb.
		 */
		static bool areColliding(const AabbForObb& aabbForObb, const Aabb& aabb);

		/**
		 * @brief Implementation of the separating axis theorem between an @p Obb and an @p Aabb.
		 */
		static bool areColliding(const Obb& obb, const Aabb& aabb);

		/**
		 * @brief Implementation of the separating axis theorem between a @p Frustum and an @p Aabb.
		 */
		static bool areColliding(const Frustum& frustum, const Aabb& aabb);

		/**
		 * @brief Returns whether a @p Ray is colliding with an @p Aabb, and the distance of the hit (if present).
		 * Implementation of the branchless slab ray-box intersection algorithm (https://tavianator.com/2011/ray_box.html).
		 * See this for a 2D visualization: https://www.geogebra.org/m/np3tnjvb
		 */
		static std::pair<bool, float> areColliding(const Ray& ray, const Aabb& aabb);

		/**
		 * @brief Returns whether a @p Ray is colliding with a @p Triangle, and the distance of the hit (if present).
		 * https://www.scratchapixel.com/lessons/3d-basic-rendering/ray-tracing-rendering-a-triangle/ray-triangle-intersection-geometric-solution.html
		 */
		static std::pair<bool, float> areColliding(const Ray& ray, const Triangle& triangle);

		/**
		 * @brief Checks whether 2 vectors are almost parallel.
		 */
		static bool almostParallel(const Vector3& lhs, const Vector3& rhs, float threshold = TOLERANCE);

		/**
		 * @brief Checks whether and OBB is "almost" an AABB.
		 */
		static bool almostAabb(const Obb& obb);

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
		static std::pair<int, int> projectedAabbExtremes(const Vector3& axis);

		/**
		 * @brief Given an axis and an @p Obb, it returns the indexes of the vertices (min and max) that are most far apart (the extremes) on the projection of an AABB to the specified axis.
		 * What happens is that the function transforms the axis into the OBB coordinate system, and then treats it like an AABB.
		 */
		static std::pair<int, int> projectedObbExtremes(const Vector3& axis, const Obb& obb);

		/**
		 * @brief Given a @p Frustum, it returns the minimum and maximum projection of its points to the specified @p axis.
		 * To do so it projects all the points and saves the minimum and maximum projections.
		 * We tried a better way (similar to the one we used fot the @p Aabb or @p Obb), but it doesn't work (look at commit ad30d3c). 
		 */
		static std::pair<float, float> projectedFrustumExtremes(const Vector3& axis, const Frustum& frustum);
	}
}
