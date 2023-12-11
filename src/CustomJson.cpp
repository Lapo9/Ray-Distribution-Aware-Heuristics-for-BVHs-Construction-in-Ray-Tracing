#include "CustomJson.h"
#include "settings.h"

void pah::to_json(json& j, const Bvh::Node& node) {
	j["id"] = (unsigned long long int) & node;
	j["aabb"] = node.aabb;
	j["leftChild"] = node.isLeaf() ? 0 : (unsigned long long int) & *node.leftChild;
	j["rightChild"] = node.isLeaf() ? 0 : (unsigned long long int) & *node.rightChild;
	
	for (auto& triangle : node.triangles) {
		j["triangles"] += (unsigned long long int) & *triangle;
	}
}

void pah::to_json(json& j, const Aabb& aabb) {
	j["min"] = aabb.min;
	j["max"] = aabb.max;
}

void pah::to_json(json& j, const Obb& obb) {
	j["center"] = obb.center;
	j["forward"] = obb.forward; //then you can (almost always) derivate right and up (considering that the up of the world is always <0,1,0>)
	j["halfSize"] = obb.halfSize;
}

void pah::to_json(json& j, const Triangle& triangle) {
	j["id"] = (unsigned long long int) &triangle;
	j["v1"] = triangle.v1;
	j["v2"] = triangle.v2;
	j["v3"] = triangle.v3;
}

void pah::to_json(json& j, const InfluenceArea& influenceArea) {
	//we try to cast to the actual type, and then call the proper function with the run-time type; if no cast works, we throw.
	//we have to do like this because C++ does NOT have multiple dispatch, therefore always this version of the function is called (with the static type InfluenceArea)
	try {
		auto& planeInfluenceArea = dynamic_cast<const PlaneInfluenceArea&>(influenceArea);
		to_json(j, planeInfluenceArea);
		return;
	} catch (const std::bad_cast&) { /* try next type */ }
	
	throw std::invalid_argument{ "The run-time type of influenceArea cannot be handled by the to_json function." };
}

void pah::to_json(json& j, const PlaneInfluenceArea& planeInfluenceArea) {
	j["type"] = "plane";
	j["plane"] = planeInfluenceArea.getPlane();
	j["size"] = planeInfluenceArea.getSize();
	j["density"] = planeInfluenceArea.getDensity();
	j["bvhRegion"] = planeInfluenceArea.getBvhRegion();
}

void pah::to_json(json& j, const BvhRegion& bvhRegion) {
	//we try to cast to the actual type, and then call the proper function with the run-time type; if no cast works, we throw.
	//we have to do like this because C++ does NOT have multiple dispatch, therefore always this version of the function is called (with the static type BvhRegion)
	try {
		auto& obbBvhRegion = dynamic_cast<const ObbBvhRegion&>(bvhRegion);
		to_json(j, obbBvhRegion);
		return;
	} catch (const std::bad_cast&) { /* try next type */ }

	throw std::invalid_argument{ "The run-time type of bvhRegion cannot be handled by the to_json function." };
}

void pah::to_json(json& j, const ObbBvhRegion& obbBvhRegion) {
	j["type"] = "obb";
	j["obb"] = obbBvhRegion.getObb();
}

void pah::to_json(json& j, const Plane& plane) {
	j["point"] = plane.getPoint();
	j["normal"] = plane.getNormal();
}

void pah::to_json(json& j, const Bvh::NodeTimingInfo& nti) {
	//log to JSON the times of this node
	TIME(j["total"] =							nti.total.count(););
	TIME(j["splittingTot"] =					nti.splittingTot.count(););
	TIME(j["computeCostTot"] =					nti.computeCostTot.count(););
	TIME(j["computeCostCount"] =				nti.computeCostCount;);
	TIME(j["computeCostAverage"] =				nti.computeCostMean().count(););
	TIME(j["splitTrianglesTot"] =				nti.splitTrianglesTot.count(););
	TIME(j["splitTrianglesCount"] =				nti.splitTrianglesCount;);
	TIME(j["splitTrianglesAverage"] =			nti.splitTrianglesMean().count(););
	TIME(j["chooseSplittingPlanesTot"] =		nti.chooseSplittingPlanesTot.count(););
	TIME(j["chooseSplittingPlanesCount"] =		nti.chooseSplittingPlanesCount;);
	TIME(j["chooseSplittingPlanesAverage"] =	nti.chooseSplittingPlanesMean().count(););
	TIME(j["shouldStopTot"] =					nti.shouldStopTot.count(););
	TIME(j["shouldStopCount"] =					nti.shouldStopCount;);
	TIME(j["shouldStopAverage"] =				nti.shouldStopMean().count(););
	TIME(j["nodesCreationTot"] =				nti.nodesCreationTot.count(););
	TIME(j["nodesCreationCount"] =				nti.nodesCreationCount;);
	TIME(j["nodesCreationAverage"] =			nti.nodesCreationMean().count(););
}

void glm::to_json(json& j, const pah::Vector3& vector) {
	j += vector.x;
	j += vector.y; 
	j += vector.z;
}

void glm::to_json(json& j, const pah::Vector2& vector) {
	j += vector.x;
	j += vector.y;
}
