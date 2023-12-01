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
		InfluenceArea();

		virtual float getProjectedArea(const Aabb& aabb) const = 0;
		virtual float getInfluence(const Aabb& aabb) const = 0;
		virtual Vector3 getRayDirection(const Aabb& aabb) const = 0;
		virtual vector<tuple<Axis, function<bool(float bestCostSoFar)>>> bestSplittingPlanes() const = 0;

		const BvhRegion& getBvhRegion() const;

	protected:
		unique_ptr<BvhRegion> bvhRegion;
	};


	class PlaneInfluenceArea : public virtual InfluenceArea {
	public:
		PlaneInfluenceArea(Plane plane, Vector2 size, float density);

		float getProjectedArea(const Aabb& aabb) const;
		float getInfluence(const Aabb& aabb) const;
		Vector3 getRayDirection(const Aabb& aabb) const;
		vector<tuple<Axis, function<bool(float bestCostSoFar)>>> bestSplittingPlanes() const;

		const Plane& getPlane() const;
		const Vector2& getSize() const;
		float getDensity() const;

	private:
		Plane plane;
		Vector2 size;
		float density;
	};
}
