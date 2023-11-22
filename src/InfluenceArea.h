#pragma once

#include <vector>
#include <tuple>
#include <functional>

#include "Utilities.h"

namespace pah{

	class InfluenceArea {
	public:
		virtual float getProjectedArea(const Aabb& aabb) const = 0;
		virtual float getInfluence(const Aabb& aabb) const = 0;
		virtual Vector3 getRayDirection(const Aabb& aabb) const = 0;
		virtual vector<tuple<Axis, function<bool(float bestCostSoFar)>>> bestSplittingPlanes() const = 0;

	private:
	};


	class PlaneInfluenceArea : public virtual InfluenceArea {
	public:
		PlaneInfluenceArea(Plane plane, Vector2 size, float density);

		float getProjectedArea(const Aabb& aabb) const;
		float getInfluence(const Aabb& aabb) const;
		Vector3 getRayDirection(const Aabb& aabb) const;
		vector<tuple<Axis, function<bool(float bestCostSoFar)>>> bestSplittingPlanes() const;

	private:
		Plane plane;
		Vector2 size;
		float density;
	};
}
