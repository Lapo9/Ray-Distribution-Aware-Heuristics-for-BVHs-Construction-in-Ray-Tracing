#include "InfluenceArea.h"

pah::InfluenceArea::InfluenceArea() {}

pah::PlaneInfluenceArea::PlaneInfluenceArea(Plane plane, Vector2 size, float density) : plane{ plane }, size{ size }, density{ density } {
}

pah::Vector3 pah::PlaneInfluenceArea::getRayDirection(const Bvh::Aabb& aabb) const {
	return aabb.center() - plane.point;
}
