#pragma once

#include <vector>
#include <memory>
#include <tuple>
#include <functional>

#include "Utilities.h"
#include "BvhRegion.h"

namespace pah {

	class InfluenceArea {
	public:
		InfluenceArea(std::unique_ptr<BvhRegion>&& bvhRegion);

		virtual float getProjectedArea(const Aabb& aabb) const = 0;
		virtual float getInfluence(const Aabb& aabb) const = 0;
		virtual Vector3 getRayDirection(const Aabb& aabb) const = 0;
		virtual std::vector<std::tuple<Axis, std::function<bool(float bestCostSoFar)>>> bestSplittingPlanes() const = 0;

		const BvhRegion& getBvhRegion() const;

	protected:
		std::unique_ptr<BvhRegion> bvhRegion;
	};


	class PlaneInfluenceArea : public virtual InfluenceArea {
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
