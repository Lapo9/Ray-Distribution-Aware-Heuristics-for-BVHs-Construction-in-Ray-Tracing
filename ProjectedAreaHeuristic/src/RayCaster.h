#pragma once

#include <vector>
#include <random>
#include <concepts>

#include "Utilities.h"
#include "TopLevel.h"
#include "Projections.h"
#include "distributions.h"

namespace pah {

	//forward declaration
	struct CumulativeRayCasterResults;

	/**
	 * @brief Class storing the results of @p RayCaster::castRays.
	 */
	struct RayCasterResults {
		int raysAmount;
		int hitsTotal;
		int missesTotal;
		int fallbackBvhSearchesTotal;
		std::unordered_map<const Bvh*, std::pair<float, int>> traversalCostForBvh;
		TIME(DurationMs timeTraversingTotal;); /**< @brief The sum of the traversal times of all the rays. */
		TIME(DurationMs timeTraversingOnlyBvhsTotal;); /**< @brief How much time it took to do the BVHs traversal. TopLevel structure traversal overhead is not included. */
		TIME(DurationMs timeTotal;); /**< @brief The total time of casting all the rays. It can be sligthly more than @p timeTotalTraversing */

		float hitsPercentage() const { return (float)hitsTotal / raysAmount; }
		float missesPercentage() const { return (float)missesTotal / raysAmount; }
		float fallbackBvhSearchesPercentage() const { return fallbackBvhSearchesTotal / (float)raysAmount; }
		float successfulFallbackBvhSearchesPercentage() const { return 1.0f - missesTotal / fallbackBvhSearchesTotal; /*remember that all misses come from a fallback search*/ }
		int nonFallbackBvhSearches() const { return raysAmount - fallbackBvhSearchesTotal; }
		float nonFallbackBvhSearchesPercentage() const { return nonFallbackBvhSearches() / (float)raysAmount; }
		std::unordered_map<const Bvh*, std::pair<float, int>> traversalCostForBvhPerRay() const { auto res = traversalCostForBvh; for (auto& e : res) e.second.first /= e.second.second; return res; }
		TIME(DurationMs timeTraversingAveragePerRay() const { return timeTraversingTotal / (float)raysAmount; })
		TIME(DurationMs timeTraversingOnlyBvhsAveragePerRay() const { return timeTraversingOnlyBvhsTotal / (float)raysAmount; })
		TIME(DurationMs timeTraversingAveragePerBvh() const { return timeTraversingTotal / (float)bvhsTraversedTotal; })
		TIME(DurationMs timeTraversingOnlyBvhsAveragePerBvh() const { return timeTraversingOnlyBvhsTotal / (float)bvhsTraversedTotal; })

		RayCasterResults& operator+=(const TopLevel::TraversalResults& rhs);
		RayCasterResults& operator+=(const Bvh::TraversalResults& rhs);
		friend CumulativeRayCasterResults operator+(const RayCasterResults& lhs, const RayCasterResults& rhs);

#define FOR_MEMBER_DO(DO) \
		DO(int, bvhsTraversed) \
		DO(int, intersectionTests) \
		DO(int, intersectionTestsWithNodes) \
		DO(int, intersectionTestsWithTriangles) \
		DO(int, intersectionTestsWhenHit) \
		DO(int, intersectionTestsWithNodesWhenHit) \
		DO(int, intersectionTestsWithTrianglesWhenHit) \
		DO(float, traversalCost)

#define FOR_NON_FALLBACK_MEMBER_DO(DO) \
		DO(int, intersectionTestsNonFallback) \
		DO(int, intersectionTestsWithNodesNonFallback) \
		DO(int, intersectionTestsWithTrianglesNonFallback) \
		DO(int, intersectionTestsWhenHitNonFallback) \
		DO(int, intersectionTestsWithNodesWhenHitNonFallback) \
		DO(int, intersectionTestsWithTrianglesWhenHitNonFallback)

#define CREATE_MEMBER(type, member) type member ## Total;
		FOR_MEMBER_DO(CREATE_MEMBER)
		FOR_NON_FALLBACK_MEMBER_DO(CREATE_MEMBER)

#define PER_RAY_AVERAGE(type, member) float member ## AveragePerRay() const { return (float) member ## Total / raysAmount; }
		FOR_MEMBER_DO(PER_RAY_AVERAGE)
		FOR_NON_FALLBACK_MEMBER_DO(PER_RAY_AVERAGE)

#define PER_BVH_AVERAGE(type, member) float member ## AveragePerBvh() const { return (float) member ## Total / bvhsTraversedTotal; }
		FOR_MEMBER_DO(PER_BVH_AVERAGE)

#define PER_BVH_NON_FALLBACK_AVERAGE(type, member) float member ## AveragePerBvh() const { return  member ## Total / (float)(bvhsTraversedTotal - fallbackBvhSearchesTotal); }
		FOR_NON_FALLBACK_MEMBER_DO(PER_BVH_NON_FALLBACK_AVERAGE)

#undef FOR_MEMBER_DO(DO)
#undef CREATE_MEMBER
#undef PER_RAY_AVERAGE
#undef PER_BVH_AVERAGE
#undef PER_BVH_NON_FALLBACK_AVERAGE
	};


	struct CumulativeRayCasterResults {
		int raysAmount;
		int rayCastersAmount;
		int hitsTotal;
		int missesTotal;
		int fallbackBvhSearchesTotal;
		std::unordered_map<const Bvh*, std::pair<float, int>> traversalCostForBvh;
		TIME(DurationMs timeTraversingTotal;);
		TIME(DurationMs timeTraversingOnlyBvhsTotal;);

		float hitsPercentage() const { return (float)hitsTotal / raysAmount; }
		float missesPercentage() const { return (float)missesTotal / raysAmount; }
		float fallbackBvhSearchesPercentage() const { return fallbackBvhSearchesTotal / (float)raysAmount; }
		float successfulFallbackBvhSearchesPercentage() const { return 1.0f - (float)missesTotal / fallbackBvhSearchesTotal; /*remember that all misses come from a BVH search*/ }
		int nonFallbackBvhSearches() const { return raysAmount - fallbackBvhSearchesTotal; }
		float nonFallbackBvhSearchesPercentage() const { return nonFallbackBvhSearches() / (float)raysAmount; }
		std::unordered_map<const Bvh*, std::pair<float, int>> traversalCostForBvhPerRay() const { auto res = traversalCostForBvh; for (auto& e : res) e.second.first /= e.second.second; return res; }
		TIME(DurationMs timeTraversingAveragePerRay() const { return timeTraversingTotal / (float)raysAmount; });
		TIME(DurationMs timeTraversingOnlyBvhsAveragePerRay() const { return timeTraversingOnlyBvhsTotal / (float)raysAmount; });
		TIME(DurationMs timeTraversingAveragePerBvh() const { return timeTraversingTotal / (float)bvhsTraversedTotal; });
		TIME(DurationMs timeTraversingOnlyBvhsAveragePerBvh() const { return timeTraversingOnlyBvhsTotal / (float)bvhsTraversedTotal; });

		CumulativeRayCasterResults& operator+=(const CumulativeRayCasterResults& rhs);
		friend CumulativeRayCasterResults operator+(CumulativeRayCasterResults lhs, const CumulativeRayCasterResults& rhs);
		CumulativeRayCasterResults& operator+=(const RayCasterResults& rhs);
		friend CumulativeRayCasterResults operator+(CumulativeRayCasterResults lhs, const RayCasterResults& rhs);

#define FOR_MEMBER_DO(DO) \
		DO(int, bvhsTraversed) \
		DO(int, intersectionTests) \
		DO(int, intersectionTestsWithNodes) \
		DO(int, intersectionTestsWithTriangles) \
		DO(int, intersectionTestsWhenHit) \
		DO(int, intersectionTestsWithNodesWhenHit) \
		DO(int, intersectionTestsWithTrianglesWhenHit) \
		DO(float, traversalCost)

#define FOR_NON_FALLBACK_MEMBER_DO(DO) \
		DO(int, intersectionTestsNonFallback) \
		DO(int, intersectionTestsWithNodesNonFallback) \
		DO(int, intersectionTestsWithTrianglesNonFallback) \
		DO(int, intersectionTestsWhenHitNonFallback) \
		DO(int, intersectionTestsWithNodesWhenHitNonFallback) \
		DO(int, intersectionTestsWithTrianglesWhenHitNonFallback) 

#define CREATE_MEMBER(type, member) type member ## Total;
		FOR_MEMBER_DO(CREATE_MEMBER)
		FOR_NON_FALLBACK_MEMBER_DO(CREATE_MEMBER)

#define PER_RAY_AVERAGE(type, member) float member ## AveragePerRay() const { return  member ## Total / (float)raysAmount; }
		FOR_MEMBER_DO(PER_RAY_AVERAGE)
		FOR_NON_FALLBACK_MEMBER_DO(PER_RAY_AVERAGE)

#define PER_BVH_AVERAGE(type, member) float member ## AveragePerBvh() const { return  member ## Total / (float)bvhsTraversedTotal; }
		FOR_MEMBER_DO(PER_BVH_AVERAGE)

#define PER_BVH_NON_FALLBACK_AVERAGE(type, member) float member ## AveragePerBvh() const { return  member ## Total / (float)(bvhsTraversedTotal - fallbackBvhSearchesTotal); }
		FOR_NON_FALLBACK_MEMBER_DO(PER_BVH_NON_FALLBACK_AVERAGE)

#undef FOR_MEMBER_DO(DO)
#undef CREATE_MEMBER
#undef PER_RAY_AVERAGE
#undef PER_BVH_AVERAGE
#undef PER_BVH_NON_FALLBACK_AVERAGE
	};


	/**
	 * @brief This class has the task to create an amount of @p Ray s and cast them against a @p TopLevel or @p Bvh., while collecting the results.
	 * @tparam Rng The random number generator used in this @p RayCaster.
	 */
	template<std::uniform_random_bit_generator Rng = std::mt19937>
	class RayCaster {
	public:
		RayCaster(const InfluenceArea& influenceArea) : influenceArea{ &influenceArea } {}

		/**
		 * @brief Generates the specified amount of @p Ray s.
		 * 
		 * @param directionTolerance How much the rays can be off the directions of the @p InfluenceArea relative to this @p RayCaster.
		 */
		virtual void generateRays(Rng& rng, unsigned int quantity, bool startRaysFromNearDepth = false, float tolerance = 0) = 0;
		
		/**
		 * @brief Casts the generated rays against a @p TopLevel structure, and collects the results.
		 */
		RayCasterResults castRays(const TopLevel& topLevel) const {
			RayCasterResults res{};
			TIME(utilities::TimeLogger timeTotal{ [&res](auto duration) {res.timeTotal = duration; } };);
			for (const auto& ray : rays) {
				res += topLevel.traverse(ray);
			}
			TIME(timeTotal.stop(););
			return res;
		}

		/**
		 * @brief Casts the generated rays against a @p Bvh, and collects the results.
		 */
		RayCasterResults castRays(const Bvh& bvh) const {
			RayCasterResults res{};
			TIME(utilities::TimeLogger timeTotal{ [&res](auto duration) {res.timeTotal = duration; } };);
			for (const auto& ray : rays) {
				res += bvh.traverse(ray);
			}
			TIME(timeTotal.stop(););
			return res;
		}

		/**
		 * @brief Returns a modifiable reference to the rays hold by this @p RayCaster.
		 */
		std::vector<Ray>& getRays() { return rays; }

	protected:
		std::vector<Ray> rays;
		const InfluenceArea* influenceArea;
	};


	template<std::uniform_random_bit_generator Rng = std::mt19937>
	class PlaneRayCaster : public RayCaster<Rng> {
	public:
		PlaneRayCaster(const PlaneInfluenceArea& planeInfluenceArea) : RayCaster<Rng>(planeInfluenceArea), 
			uniformRectangleDistribution{ -planeInfluenceArea.getSize().x, planeInfluenceArea.getSize().x, -planeInfluenceArea.getSize().y, planeInfluenceArea.getSize().y } {}
		
		void generateRays(Rng& rng, unsigned int quantity, bool startRaysFromNearDepth = false, float directionTolerance = 0) override {
			this->rays = std::vector<Ray>{}; //clear the vector
			this->rays.reserve(quantity); //reserve space for the elements (to avoid reallocations)

			const PlaneInfluenceArea* planeInfluenceArea = dynamic_cast<const PlaneInfluenceArea*>(this->influenceArea);
			const auto& planeSize = planeInfluenceArea->getSize();

			// point in world space * view matrix = point in cam.space <==> point in cam. space * (view matrix)^-1 = point in world space
			// P in space A * M = P in space B <==> P in space B * M^-1 = P in space A
			// Where M is the matrix whose first 3 rows are the base vectors of space B as seen from space A (excluding the last element of each row that is the translation component)
			// Visualization at: https://www.geogebra.org/m/ndynxgqr
			auto[right, up, forward] = utilities::rightHandCoordinatesSystem(planeInfluenceArea->getPlane().getNormal());
			//at the moment all the OBBs have roll == 0, therefore we build the basis of the space of the plane with zero roll (as if it was a standard camera). This may change in the future.
			Matrix4 changeOfCoords = glm::inverse(projection::computeChangeOfCoordinatesMatrix(right, up, forward, planeInfluenceArea->getPlane().getPoint()));
			//basically, this matrix allow as to go from the coordinate system centered at the center of the plane and with the z-axis perpendicular to the plane (which is the one we use to build the OBB) to the world space.
			//therefore, now, we can get a random point on the plane, get its coordinates in world space and use those to create the ray (the direction is always perpendicular to the plane itself).

			distributions::UniformSphereCapDistribution directionDistribution{ directionTolerance, forward }; //distribution to give variance to the direction of the rays
			//not all rays spawn on the plane, they may spawn to a certain distance from it (up to the z-length of the region)
			std::uniform_real_distribution<> originDepthDistribution{ 0, planeInfluenceArea->getFar() };
			
			//eventually, create the rays
			for (int i = 0; i < quantity; ++i) {
				Vector3 direction = directionDistribution(rng);
				float depth = originDepthDistribution(rng);
				Vector3 origin = changeOfCoords * Vector4{ uniformRectangleDistribution(rng), 0.0f, 1.0f };
				if(!startRaysFromNearDepth) origin += direction * depth;
				this->rays.emplace_back(origin, direction);
			}
		}

	private:
		distributions::UniformRectangleDistribution uniformRectangleDistribution;
	};


	template<std::uniform_random_bit_generator Rng = std::mt19937>
	class PointRayCaster : public RayCaster<Rng> {
	public:
		PointRayCaster(const PointInfluenceArea& pointInfluenceArea) : RayCaster<Rng>(pointInfluenceArea), 
			directionDistribution{ pointInfluenceArea.getPov().fovX/2.0f, pointInfluenceArea.getPov().fovY/2.0f, pointInfluenceArea.getPov().getDirection() } {}

		void generateRays(Rng& rng, unsigned int quantity, bool startRaysFromNearDepth = false, float originTolerance = 0) override {
			this->rays = std::vector<Ray>{}; //clear the vector
			this->rays.reserve(quantity); //reserve space for the elements (to avoid reallocations)

			const PointInfluenceArea* pointInfluenceArea = dynamic_cast<const PointInfluenceArea*>(this->influenceArea);

			//read comments in the analogue section in PlaneRayCaster::generateRays
			auto [right, up, forward] = utilities::rightHandCoordinatesSystem(pointInfluenceArea->getPov().getDirection());
			Matrix4 changeOfCoords = glm::inverse(projection::computeChangeOfCoordinatesMatrix(right, up, forward, pointInfluenceArea->getPov().position));

			//rays can start a bit off the real origin (to give variance)
			distributions::UniformDiskDistribution originDistribution{ originTolerance };
			//the region where rays spawns doesn't start at the origin, but between the near and far plane of the frustum. 0.f means that the ray starts at the near plane, 1.f at the far plane
			std::uniform_real_distribution<> originDepthDistributionPercentage{ 0.f, 1.f };

			for (int i = 0; i < quantity; ++i) {
				Vector3 direction = directionDistribution(rng);
				Vector3 origin = changeOfCoords * Vector4{ originDistribution(rng), 0.0f, 1.0f };

				// now we compute the distance on the ray where the ray intersects the near and far planes of the frustum, and, based on the depth percentage, we compute where the ray origin should be
				Ray rayFromOrigin = { origin, direction };
				Pov pov = pointInfluenceArea->getPov();
				auto [nearPlaneDist, farPlaneDist] = pointInfluenceArea->getNearFar();
				float distNear = collisionDetection::areColliding(rayFromOrigin, Plane{ pov.position + pov.getDirection() * nearPlaneDist, pov.getDirection() }).distance;
				float distFar = collisionDetection::areColliding(rayFromOrigin, Plane{ pov.position + pov.getDirection() * farPlaneDist, pov.getDirection() }).distance;

				float depthPercentage = startRaysFromNearDepth ? 0.f : originDepthDistributionPercentage(rng);
				float depth = depthPercentage * (distFar - distNear) + distNear;

				origin += direction * depth;
				this->rays.emplace_back(origin, direction);
			}

			//TODO yet to be tested
		}

	private:
		distributions::UniformSquareSphereCapDistribution directionDistribution;
	};
}
