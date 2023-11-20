#pragma once

#include "Utilities.h"

namespace pah{
	struct Plane {
		Vector3 point;
		Vector3 normal;
	};

	class InfluenceArea {
	public:
		InfluenceArea();

		virtual float getProjectedArea(const Aabb& aabb) const = 0;
		virtual float getInfluence(const Aabb& aabb) const = 0;
		virtual Vector3 getRayDirection(const Aabb& aabb) const = 0;
		virtual ChooseSplittingPlanesReturnType bestSplittingPlanes() const = 0;

	private:
	};


	class PlaneInfluenceArea : public virtual InfluenceArea {
	public:
		PlaneInfluenceArea(Plane plane, Vector2 size, float density);

		float getProjectedArea(const Aabb& aabb);
		float getInfluence(const Aabb& aabb);
		Vector3 getRayDirection(const Aabb& aabb);
		ChooseSplittingPlanesReturnType bestSplittingPlanes();

	private:
		Plane plane;
		Vector2 size;
		float density;
	};
}
