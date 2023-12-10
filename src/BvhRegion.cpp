#include "BvhRegion.h"

pah::ObbBvhRegion::ObbBvhRegion(const Vector3& center, const Vector3& halfSize, const Vector3& forward) : obb{ center, halfSize, forward }, enclosingAabb{ Obb::enclosingAabb(obb) } {
}

bool pah::ObbBvhRegion::isInside(const Vector3& point) const {
	//check if the point is inside the AABB (cheap), if it is, check the OBB
	if (isInsideAabb(point)) return isInsideObb(point);
	return false;
}

const pah::Obb& pah::ObbBvhRegion::getObb() const {
	return obb;
}

bool pah::ObbBvhRegion::isInsideObb(const Vector3& point) const {
	using namespace glm;

	Vector3 d = point - obb.center;
	//we project the distance vector between the point and the center in the reference system of the OBB, and we check the distances are < than the half size in all directions
	return 
		abs(dot(d, obb.right)) <= obb.halfSize.x &&
		abs(dot(d, obb.up)) <= obb.halfSize.y &&
		abs(dot(d, obb.forward)) <= obb.halfSize.z;
}

bool pah::ObbBvhRegion::isInsideAabb(const Vector3& point) const {
	return
		point.x >= enclosingAabb.min.x &&
		point.y >= enclosingAabb.min.y &&
		point.z >= enclosingAabb.min.z &&
		point.x <= enclosingAabb.max.x &&
		point.y <= enclosingAabb.max.y &&
		point.z <= enclosingAabb.max.z;
}


