#include "InfluenceArea.h"

#include "Projections.h"

using namespace std;
using namespace pah;


// ======| InfluenceArea |======
pah::InfluenceArea::InfluenceArea(std::unique_ptr<Region>&& region) : bvhRegion{ std::move(region) } {}

const pah::Region& pah::InfluenceArea::getBvhRegion() const {
	return *bvhRegion;
}


// ======| PlaneInfluenceArea |======
pah::PlaneInfluenceArea::PlaneInfluenceArea(Plane plane, Vector2 size, float density, float forwardSize)
	: InfluenceArea{ make_unique<AabbForObb>(plane.getPoint() + plane.getNormal() * (forwardSize / 2.0f), Vector3{size.x, size.y, forwardSize / 2.0f}, plane.getNormal()) },
	plane{ plane }, size{ size }, density{ density } {
}

float pah::PlaneInfluenceArea::getProjectedArea(const Aabb& aabb) const {
	return projection::orthographic::computeProjectedArea(aabb, plane);
}

float pah::PlaneInfluenceArea::getInfluence(const Aabb& aabb) const {
	return density;
}

Vector3 pah::PlaneInfluenceArea::getRayDirection(const Aabb& aabb) const {
	//return aabb.center() - plane.getPoint(); //TODO test and remove
	return plane.getNormal();
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


// ======| PointInfluenceArea |======
pah::PointInfluenceArea::PointInfluenceArea(Pov pov, float far, float near, float fovX, float fovY, float density)
	: InfluenceArea{ make_unique<Frustum>(pov, far, near, fovX, fovY) } {}

float pah::PointInfluenceArea::getProjectedArea(const Aabb& aabb) const
{
	//we want to project to a frustum that points to the AABB.
	//indeed the projected area should approximate the number of rays that hit the AABB, but by projecting it to the near plane, objects far from the z axis tend to get distorted.
	//basically we want the solid angle.
	Pov centeredPov{ pov.getPosition(), aabb.center() - pov.getPosition() };
	return projection::perspective::computeProjectedArea(aabb, centeredPov);
}

float pah::PointInfluenceArea::getInfluence(const Aabb& aabb) const
{
	return density;
}

Vector3 pah::PointInfluenceArea::getRayDirection(const Aabb& aabb) const
{
	return aabb.center() - pov.getPosition();
}

std::vector<std::tuple<Axis, std::function<bool(float bestCostSoFar)>>> pah::PointInfluenceArea::bestSplittingPlanes() const
{
	throw logic_error("Function not implemented yet!");
}
