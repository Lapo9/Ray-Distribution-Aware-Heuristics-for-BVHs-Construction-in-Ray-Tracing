#include "InfluenceArea.h"

pah::InfluenceArea::InfluenceArea() {}

pah::PlaneInfluenceArea::PlaneInfluenceArea(Plane plane, Vector2 size, float density) : plane{ plane }, size{ size }, density{ density } {
}

float pah::PlaneInfluenceArea::getProjectedArea(const Aabb& aabb) const {
	return 0.0f;
}

pah::Vector3 pah::PlaneInfluenceArea::getRayDirection(const Aabb& aabb) const {
	return aabb.center() - plane.point;
}
