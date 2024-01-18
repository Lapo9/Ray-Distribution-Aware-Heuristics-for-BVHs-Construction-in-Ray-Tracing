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
pah::PlaneInfluenceArea::PlaneInfluenceArea(Plane plane, Vector2 size, float forwardSize, float density)
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

bool pah::PlaneInfluenceArea::isDirectionAffine(const Vector3& direction, float tolerance) const {
	return collisionDetection::almostParallel(direction, plane.getNormal(), tolerance);
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

// Look at the comment of PointInfluenceArea::planePatch to understand how we built it. The order of the point is such that it creates a counterclockwise rectangle.
pah::PointInfluenceArea::PointInfluenceArea(Pov pov, float far, float near, float density)
	: InfluenceArea{ make_unique<Frustum>(pov, far, near) }, pov{ pov }, density{ density },
	planePatch{
		dynamic_cast<Frustum&>(*bvhRegion).getEdgesDirections()[2],
		dynamic_cast<Frustum&>(*bvhRegion).getEdgesDirections()[4],
		dynamic_cast<Frustum&>(*bvhRegion).getEdgesDirections()[5],
		dynamic_cast<Frustum&>(*bvhRegion).getEdgesDirections()[3]
	} { }

float pah::PointInfluenceArea::getProjectedArea(const Aabb& aabb) const
{
	//we want to project to a frustum that points to the AABB.
	//indeed the projected area should approximate the number of rays that hit the AABB, but by projecting it to the near plane, objects far from the z axis tend to get distorted.
	//basically we want the solid angle.
	Pov centeredPov{ pov.position, aabb.center() - pov.position, pov.fovX, pov.fovY };
	return projection::perspective::computeProjectedArea(aabb, centeredPov);
}

float pah::PointInfluenceArea::getInfluence(const Aabb& aabb) const{
	return density;
}

Vector3 pah::PointInfluenceArea::getRayDirection(const Aabb& aabb) const{
	return aabb.center() - pov.position;
}

bool pah::PointInfluenceArea::isDirectionAffine(const Vector3& direction, float tolerance) const {
	return collisionDetection::areColliding(Ray{ Vector3{0,0,0}, direction }, planePatch).hit;
}

std::vector<std::tuple<Axis, std::function<bool(float bestCostSoFar)>>> pah::PointInfluenceArea::bestSplittingPlanes() const{
	throw logic_error("Function not implemented yet!");
}

const pah::Pov& pah::PointInfluenceArea::getPov() const {
	return pov;
}

float pah::PointInfluenceArea::getDensity() const {
	return density;
}
