#include "InfluenceArea.h"

using namespace std;
using namespace pah;


pah::InfluenceArea::InfluenceArea(std::unique_ptr<BvhRegion>&& bvhRegion) : bvhRegion{ std::move(bvhRegion) } {}

const pah::BvhRegion& pah::InfluenceArea::getBvhRegion() const {
	return *bvhRegion;
}


pah::PlaneInfluenceArea::PlaneInfluenceArea(Plane plane, Vector2 size, float density, float forwardSize) 
	: InfluenceArea{ make_unique<ObbBvhRegion>(plane.getPoint() + plane.getNormal() * forwardSize/2.0f, Vector3{size.x, size.y, forwardSize/2.0f}, plane.getNormal()) },
	plane{ plane }, size{ size }, density{ density } {}

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

const Plane& pah::PlaneInfluenceArea::getPlane() const {
	return plane;
}

const Vector2& pah::PlaneInfluenceArea::getSize() const {
	return size;
}

float pah::PlaneInfluenceArea::getDensity() const {
	return density;
}

