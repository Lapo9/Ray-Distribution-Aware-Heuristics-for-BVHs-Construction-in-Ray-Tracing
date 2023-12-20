#pragma once

#include <vector>
#include <memory>
#include <functional>
#include <utility>

#include "Utilities.h"
#include "Regions.h"

namespace pah {

	class InfluenceArea {
	public:
		InfluenceArea(std::unique_ptr<Region>&& region) : bvhRegion{ std::move(bvhRegion) } {}

		virtual float getProjectedArea(const Aabb& aabb) const = 0;
		virtual float getInfluence(const Aabb& aabb) const = 0;
		virtual Vector3 getRayDirection(const Aabb& aabb) const = 0;
		virtual std::vector<std::tuple<Axis, std::function<bool(float bestCostSoFar)>>> bestSplittingPlanes() const = 0;

		const Region& getBvhRegion() const;

	protected:
		std::unique_ptr<Region> bvhRegion;
	};


	class PlaneInfluenceArea : public InfluenceArea {
	public:
		PlaneInfluenceArea(Plane plane, Vector2 size, float density, float forwardSize);

		float getProjectedArea(const Aabb& aabb) const override;
		float getInfluence(const Aabb& aabb) const override;
		Vector3 getRayDirection(const Aabb& aabb) const override;
		std::vector<std::tuple<Axis, std::function<bool(float bestCostSoFar)>>> bestSplittingPlanes() const override;

		const Plane& getPlane() const;
		const Vector2& getSize() const;
		float getDensity() const;


	private:
		Plane plane;
		Vector2 size;
		float density;
	};
}
