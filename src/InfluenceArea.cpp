#include "InfluenceArea.h"

using namespace std;
using namespace pah;

pah::PlaneInfluenceArea::PlaneInfluenceArea(Plane plane, Vector2 size, float density) : plane{ plane }, size{ size }, density{ density } {
}

float pah::PlaneInfluenceArea::getProjectedArea(const Aabb& aabb) const {
	return projection::orthographic::computeProjectedArea(aabb, plane);
}

float pah::PlaneInfluenceArea::getInfluence(const Aabb& aabb) const {
	return density;
}

Vector3 pah::PlaneInfluenceArea::getRayDirection(const Aabb& aabb) const {
	return aabb.center() - plane.getPoint();
}

vector<tuple<Axis, function<bool(float bestCostSoFar)>>> pah::PlaneInfluenceArea::bestSplittingPlanes() const {
	throw logic_error("Function not implemented yet!");
}
