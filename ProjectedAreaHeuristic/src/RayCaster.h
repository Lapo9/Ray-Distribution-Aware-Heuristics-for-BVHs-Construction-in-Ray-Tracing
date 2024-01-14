#pragma once

#include <vector>
#include <random>
#include <concepts>

#include "Utilities.h"
#include "TopLevel.h"
#include "Projections.h"

namespace pah {
	struct RayCasterResults {
		int raysAmount;
		int totalBvhsTraversed;
		int totalHits;
		int totalMisses;
		int totalIntersections;
		int totalIntersectionsWithNodes;
		int totalIntersectionsWithTriangles;
		int totalIntersectionsWhenHit;
		int totalIntersectionsWithNodesWhenHit;
		int totalIntersectionsWithTrianglesWhenHit;
		TIME(DurationMs totalTimeTraversing;); /**< @brief The sum of the traversal times of all the rays. */
		TIME(DurationMs totalTime;); /**< @brief The total time of casting all the rays. It can be sligthly more than @p totalTimeTraversing */

		RayCasterResults& operator+=(const TopLevel::TraversalResults& rhs) {
			raysAmount++;
			totalBvhsTraversed += rhs.bvhsTraversed;
			totalHits += rhs.hit();
			totalMisses += !rhs.hit();
			totalIntersections += rhs.intersectionsTotal;
			totalIntersectionsWithNodes += rhs.intersectionsWithNodes;
			totalIntersectionsWithTriangles += rhs.intersectionsWithTriangles;
			totalIntersectionsWhenHit = rhs.intersectionsWhenHit;
			totalIntersectionsWithNodesWhenHit = rhs.intersectionsWithNodesWhenHit;
			totalIntersectionsWithTrianglesWhenHit = rhs.intersectionsWithTrianglesWhenHit;
			TIME(totalTimeTraversing += rhs.traversalTime;);

			return *this;
		}

		RayCasterResults& operator+=(const Bvh::TraversalResults& rhs) {
			raysAmount++;
			totalBvhsTraversed++;
			totalHits += rhs.hit();
			totalMisses += !rhs.hit();
			totalIntersections += rhs.intersectionsTotal;
			totalIntersectionsWithNodes += rhs.intersectionsWithNodes;
			totalIntersectionsWithTriangles += rhs.intersectionsWithTriangles;
			TIME(totalTimeTraversing += rhs.traversalTime;);
			if (rhs.hit()) {
				totalIntersectionsWhenHit = rhs.intersectionsTotal;
				totalIntersectionsWithNodesWhenHit = rhs.intersectionsWithNodes;
				totalIntersectionsWithTrianglesWhenHit = rhs.intersectionsWithTriangles;
			}
			return *this;
		}

		float averageBvhsTraversed() const {
			return (float)totalBvhsTraversed / raysAmount;
		}

		float hitsPercentage() const {
			return (float)totalHits / raysAmount;
		}

		float missesPercentage() const {
			return (float)totalMisses / raysAmount;
		}

		float averageIntersections() const {
			return (float)totalIntersections / raysAmount;
		}

		float averageIntersectionsWithNodes() const {
			return (float)totalIntersectionsWithNodes / raysAmount;
		}

		float averageIntersectionsWithTriangles() const {
			return (float)totalIntersectionsWithTriangles / raysAmount;
		}

		float averageIntersectionsWhenHit() const {
			return (float)totalIntersectionsWhenHit / raysAmount;
		}

		float averageIntersectionsWithNodesWhenHit() const {
			return (float)totalIntersectionsWithNodesWhenHit / raysAmount;
		}

		float averageIntersectionsWithTrianglesWhenHit() const {
			return (float)totalIntersectionsWithTrianglesWhenHit / raysAmount;
		}

		TIME(DurationMs averageTimeTraversing() const {
			return totalTimeTraversing / (float)raysAmount;
		})
	};

	template<std::uniform_random_bit_generator Rng = std::mt19937>
	class RayCaster {
	public:
		RayCaster(const InfluenceArea& influenceArea) : influenceArea{ &influenceArea } {}

		/**
		 * @brief Generates the specified amount of @p Ray s.
		 * 
		 * @param directionTolerance How much the rays can be off the directions of the @p InfluenceArea relative to this @p RayCaster.
		 */
		virtual void generateRays(const Rng& rng, unsigned int quantity, float directionTolerance = 0) = 0;
		
		/**
		 * @brief Casts the generated rays against a @p TopLevel structure, and collects the results.
		 */
		RayCasterResults castRays(const TopLevel& topLevel) const {
			RayCasterResults res{};
			TIME(utilities::TimeLogger totalTime{ [res&](auto duration) {res.totalTime = duration; } };);
			for (const auto& ray : rays) {
				res += topLevel.traverse(ray);
			}
			TIME(totalTime.stop(););
		}

		/**
		 * @brief Casts the generated rays against a @p Bvh, and collects the results.
		 */
		RayCasterResults castRays(const Bvh& bvh) const {
			RayCasterResults res{};
			for (const auto& ray : rays) {
				res += bvh.traverse(ray);
			}
		}

	protected:
		std::vector<Ray> rays;
		InfluenceArea* influenceArea;
	};


	template<std::uniform_random_bit_generator Rng = std::mt19937>
	class PlaneRayCaster : public RayCaster<Rng> {
	public:
		PlaneRayCaster(const PlaneInfluenceArea& planeInfluenceArea) : RayCaster<Rng>(planeInfluenceArea) {}
		
		void generateRays(const Rng& rng, unsigned int quantity, float directionTolerance = 0) override {
			PlaneInfluenceArea* planeInfluenceArea = dynamic_cast<PlaneInfluenceArea*>(influenceArea);
			const auto& planeSize = planeInfluenceArea->getSize();
			UniformRectangleDistribution distr{ -planeSize.x, planeSize.y, -planeSize.y, planeSize.y };

			// point in world space * view matrix = point in cam.space <==> point in cam. space * (view matrix)^-1 = point in world space
			// P in space A * M = P in space B <==> P in space B * M^-1 = P in space A
			// Where M is the matrix whose first 3 rows are the base vectors of space B as seen from space A (excluding the last element of each row that is the translation component)
			// Visualization at: https://www.geogebra.org/m/ndynxgqr
			Vector3 forward = planeInfluenceArea->getPlane().getNormal();
			Vector3 right = glm::cross(Vector3{ 0,1,0 }, forward);
			Vector3 up = glm::cross(forward, right);
			//at the moment all the OBBs have roll == 0, therefore we build the basis of the space of the plane with zero roll (as if it was a standard camera). This may change in the future.
			Matrix4 changeOfCoords = glm::inverse(projection::computeChangeOfCoordinatesMatrix(right, up, forward, planeInfluenceArea->getPlane().getPoint()));
			//basically, this matrix allow as to go from the coordinate system centered at the center of the plane and with the z-axis perpendicular to the plane (which is the one we use to build the OBB) to the world space.
			//therefore, now, we can get a random point on the plane, get its coordinates in world space and use those to create the ray (the direction is always perpendicular to the plane itself).


		}
	};


	template<std::uniform_random_bit_generator Rng = std::mt19937>
	class PointRayCaster : public RayCaster<Rng> {
	public:
		PointRayCaster(const PointInfluenceArea& pointInfluenceArea) : RayCaster<Rng>(pointInfluenceArea) {}

		void generateRays(const Rng& rng, unsigned int quantity, float directionTolerance = 0) override {
			//TODO this won't be easy
		}
	};
}
