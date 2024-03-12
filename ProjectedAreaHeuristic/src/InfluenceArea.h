#pragma once

#include <vector>
#include <memory>
#include <tuple>
#include <functional>

#include "Utilities.h"
#include "Regions.h"

namespace pah {

	/**
	 * @brief Defines an area where there is a certain distribution of rays.
	 * Each @p InfluenceArea has an associated @p Region.
	 */
	class InfluenceArea {
	public:
		InfluenceArea(std::unique_ptr<Region>&& region);

		/**
		 * @brief Given an @p Aabb, returns the area projected by this @p Aabb on this @p InfluenceArea.
		 */
		virtual float getProjectedArea(const Aabb& aabb) const = 0;

		/**
		 * @brief Given an @p Aabb, returns the influence that this @p InfluenceArea has on it.
		 * This may depend on many factors, such as the amount of rays in the area or the distance.
		 */
		virtual float getInfluence(const Aabb& aabb) const = 0;

		/**
		 * @brief Given an @p Aabb, returns the direction that the ray hitting the center of the AABB has.
		 */
		virtual Vector3 getRayDirection(const Aabb& aabb) const = 0;

		/**
		 * @brief Returns whether a direction is "affine" to this influence area.
		 */
		virtual bool isDirectionAffine(const Ray& ray, float tolerance) const = 0;

		// TODO probably this will be removed. It should return the best way to split the AABB
		virtual std::vector<std::tuple<Axis, std::function<bool(float bestCostSoFar)>>> bestSplittingPlanes() const = 0;

		/**
		 * @brief Returns the associated @p Region.
		 */
		const Region& getBvhRegion() const;

	protected:
		std::unique_ptr<Region> bvhRegion;
	};


	/**
	 * @brief This @p InfluenceArea can be represented as a rectangle on a plane. Rays are perpendicular to this plane.
	 */
	class PlaneInfluenceArea : public InfluenceArea {
	public:
		PlaneInfluenceArea(Plane plane, Vector2 size, float forwardSize, float density);

		float getProjectedArea(const Aabb& aabb) const override;
		float getInfluence(const Aabb& aabb) const override;
		Vector3 getRayDirection(const Aabb& aabb) const override;
		bool isDirectionAffine(const Ray& ray, float tolerance) const override;
		std::vector<std::tuple<Axis, std::function<bool(float bestCostSoFar)>>> bestSplittingPlanes() const override;

		const Plane& getPlane() const;
		const Vector2& getSize() const;
		float getFar() const;
		float getDensity() const;


	private:
		Plane plane;
		Vector2 size;
		float farPlane;
		float density;
		Matrix4 viewMatrix; //stores the view matrix for this plane, so that we can avoid calculating it each time
	};


	class PointInfluenceArea : public InfluenceArea {
	public:
		PointInfluenceArea(Pov pov, float far, float near, float density);

		float getProjectedArea(const Aabb& aabb) const override;
		float getInfluence(const Aabb& aabb) const override;
		Vector3 getRayDirection(const Aabb& aabb) const override;
		bool isDirectionAffine(const Ray& ray, float tolerance) const override;
		std::vector<std::tuple<Axis, std::function<bool(float bestCostSoFar)>>> bestSplittingPlanes() const override;

		const Pov& getPov() const;
		const std::pair<float, float> getNearFar() const;
		float getDensity() const;

	private:
		float density;
		Pov pov;
		float nearPlane, farPlane;
	};
}
