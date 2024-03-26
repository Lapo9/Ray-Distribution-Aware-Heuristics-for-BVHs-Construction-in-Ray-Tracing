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
pah::PlaneInfluenceArea::PlaneInfluenceArea(Plane plane, float forwardSize, float density)
	: InfluenceArea{ make_unique<AabbForObb>(plane.getPoint() + plane.getNormal() * (forwardSize / 2.0f), Vector3{plane.width, plane.height, forwardSize / 2.0f}, plane.getNormal()) },
	viewProjectionMatrix{ projection::computeViewMatrix(plane.getPoint(), plane.getNormal()) },
	plane{ plane }, density{ density }, farPlane{ forwardSize } {
	if constexpr (!FAST_ORTHOGRAPHIC_PROJECTIONS) viewProjectionMatrix *= projection::computeOrthographicMatrix(-plane.width, plane.width, -plane.height, plane.height);
}

float pah::PlaneInfluenceArea::getProjectedArea(const Aabb& aabb) const {
	return projection::orthographic::computeProjectedArea(aabb, viewProjectionMatrix);
}

std::vector<Vector2> pah::PlaneInfluenceArea::getProjectedHull(const Aabb& aabb) const {
	const auto& contourPoints = projection::orthographic::findContourPoints(aabb, plane.getNormal());
	return projection::orthographic::projectPoints(contourPoints, plane);
}

float pah::PlaneInfluenceArea::getInfluence(const Aabb& aabb) const {
	return density;
}

Vector3 pah::PlaneInfluenceArea::getRayDirection(const Aabb& aabb) const {
	return plane.getNormal();
}

float pah::PlaneInfluenceArea::getProjectionPlaneArea() const {
	if constexpr (!FAST_ORTHOGRAPHIC_PROJECTIONS) return 4; //since we project to the canonical view volume, the projected area of an object completely filling the view is always (1-(-1)) * (1-(-1)) = 4
	else return plane.width * plane.height * 4;
}

std::vector<Vector2> pah::PlaneInfluenceArea::getProjectionPlaneHull() const {
	if constexpr (!FAST_ORTHOGRAPHIC_PROJECTIONS) return { {-1,-1},{1,-1},{1,1},{-1,1} };
	return { {-plane.width,-plane.height}, {plane.width,-plane.height}, {plane.width,plane.height}, {-plane.width,plane.height} };
}

bool pah::PlaneInfluenceArea::isDirectionAffine(const Ray& ray, float tolerance) const {
	return collisionDetection::almostParallel(ray.getDirection(), plane.getNormal(), tolerance);
}

std::vector<std::tuple<pah::Axis, std::function<bool(float bestCostSoFar)>>> pah::PlaneInfluenceArea::bestSplittingPlanes() const {
	throw logic_error("Function not implemented yet!");
}

const pah::Plane& pah::PlaneInfluenceArea::getPlane() const {
	return plane;
}

const pah::Vector2& pah::PlaneInfluenceArea::getSize() const {
	return { plane.width, plane.height };
}

float pah::PlaneInfluenceArea::getFar() const {
	return farPlane;
}

float pah::PlaneInfluenceArea::getDensity() const {
	return density;
}


// ======| PointInfluenceArea |======

// Look at the comment of PointInfluenceArea::planePatch to understand how we built it. The order of the point is such that it creates a counterclockwise rectangle.
pah::PointInfluenceArea::PointInfluenceArea(Pov pov, float far, float near, float density)
	: InfluenceArea{ make_unique<Frustum>(pov, far, near) }, pov{ pov }, density{ density }, nearPlane{ near }, farPlane{ far } {
}

float pah::PointInfluenceArea::getProjectedArea(const Aabb& aabb) const
{
	//we want to project to a frustum that points to the AABB.
	//indeed the projected area should approximate the number of rays that hit the AABB, but by projecting it to the near plane, objects far from the z axis tend to get distorted.
	//basically we want the solid angle.
	//Pov centeredPov{ pov.position, aabb.center() - pov.position, pov.fovX, pov.fovY };
	return projection::perspective::computeProjectedArea(aabb, pov);
}

std::vector<Vector2> pah::PointInfluenceArea::getProjectedHull(const Aabb& aabb) const {
	const auto& contourPoints = projection::perspective::findContourPoints(aabb, pov.position);
	return projection::perspective::projectPoints(contourPoints, pov);
}

float pah::PointInfluenceArea::getInfluence(const Aabb& aabb) const{
	return density;
}

Vector3 pah::PointInfluenceArea::getRayDirection(const Aabb& aabb) const{
	return aabb.center() - pov.position;
}

float pah::PointInfluenceArea::getProjectionPlaneArea() const {
	return 4; //since we project to the canonical view volume, the projected area of an object completely filling the view is always (1-(-1)) * (1-(-1)) = 4
}

std::vector<Vector2> pah::PointInfluenceArea::getProjectionPlaneHull() const {
	return { {-1,-1},{1,-1},{1,1},{-1,1} };
}

bool pah::PointInfluenceArea::isDirectionAffine(const Ray& ray, float tolerance) const {
	// get the focal point of this point influence area and connect it with the ray origin; then compare the directions of this segment and the ray
	// we assume that it is true that the ray origin is inside the influence area
	Vector3 povOriginDirection = ray.getOrigin() - pov.position;
	if (glm::length(povOriginDirection) <= TOLERANCE && glm::length(povOriginDirection) >= -TOLERANCE) return true;
	return collisionDetection::almostParallel(ray.getDirection(), povOriginDirection, tolerance);
}

std::vector<std::tuple<Axis, std::function<bool(float bestCostSoFar)>>> pah::PointInfluenceArea::bestSplittingPlanes() const{
	throw logic_error("Function not implemented yet!");
}

const pah::Pov& pah::PointInfluenceArea::getPov() const {
	return pov;
}

const std::pair<float, float> pah::PointInfluenceArea::getNearFar() const {
	return { nearPlane, farPlane };
}

float pah::PointInfluenceArea::getDensity() const {
	return density;
}
