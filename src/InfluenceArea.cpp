#include "InfluenceArea.h"

#include "Utilities.h"
#include "Projections.h"

using namespace std;
using namespace pah;

const pah::Region& pah::InfluenceArea::getBvhRegion() const {
	return *bvhRegion;
}

pah::PlaneInfluenceArea::PlaneInfluenceArea(Plane plane, Vector2 size, float density, float forwardSize)
	: InfluenceArea{ make_unique<AabbForObb>(plane.getPoint() + plane.getNormal() * forwardSize / 2.0f, Vector3{size.x, size.y, forwardSize / 2.0f}, plane.getNormal()) },
	plane{ plane }, size{ size }, density{ density } {
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

std::vector<std::tuple<pah::Axis, std::function<bool(float bestCostSoFar)>>> pah::PlaneInfluenceArea::bestSplittingPlanes() const {
	throw logic_error("Function not implemented yet!");
}

const pah::Plane& pah::PlaneInfluenceArea::getPlane() const {
	return plane;
}

const pah::Vector2& pah::PlaneInfluenceArea::getSize() const {
	return size;
}

float pah::PlaneInfluenceArea::getDensity() const {
	return density;
}
