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

void pah::to_json(json& j, const Region& region) {
	//we try to cast to the actual type, and then call the proper function with the run-time type; if no cast works, we throw.
	//we have to do like this because C++ does NOT have multiple dispatch, therefore always this version of the function is called (with the static type Region)
	try {
		auto& aabbRegion = dynamic_cast<const Aabb&>(region);
		to_json(j, aabbRegion);
		return;
	} catch (const std::bad_cast&) {}
	try {
		auto& obbRegion = dynamic_cast<const Obb&>(region);
		to_json(j, obbRegion);
		return;
	} catch (const std::bad_cast&) {}
	try {
		auto& aabbForObbRegion = dynamic_cast<const AabbForObb&>(region);
		to_json(j, aabbForObbRegion);
		return;
	} catch (const std::bad_cast&) {}

	throw std::invalid_argument{ "The run-time type of region cannot be handled by the to_json function." };
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

void pah::to_json(json& j, const AabbForObb& aabbForObb) {
	j["type"] = "obb";
	j["aabb"] = aabbForObb.aabb;
	j["obb"] = aabbForObb.obb;
}

void pah::to_json(json& j, const Frustum& frustum) {
	j["type"] = "frustum";
	j["matrix"] = frustum.getMatrix();
	j["vertices"] = frustum.getPoints();
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

void pah::to_json(json& j, const pah::TopLevelOctree::Node& node) {
	j["id"] = (unsigned long long int) &node;
	j["aabb"] = node.aabb;
	for (const auto& child : node.children) {
		j["children"] += (long long int) &*child;
	}
	for (const auto& bvhPtr : node.bvhs) {
		j["bvhs"] += (long long int) bvhPtr;
	}
	j["isLeaf"] = node.isLeaf();
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

void glm::to_json(json& j, const pah::Matrix4& matrix) {
	j += matrix[0][0];
	j += matrix[0][1];
	j += matrix[0][2];
	j += matrix[0][3];

	j += matrix[1][0];
	j += matrix[1][1];
	j += matrix[1][2];
	j += matrix[1][3];

	j += matrix[2][0];
	j += matrix[2][1];
	j += matrix[2][2];
	j += matrix[2][3];

	j += matrix[3][0];
	j += matrix[3][1];
	j += matrix[3][2];
	j += matrix[3][3];
}

void glm::to_json(json& j, const pah::Matrix3& matrix) {
	j += matrix[0][0];
	j += matrix[0][1];
	j += matrix[0][2];

	j += matrix[1][0];
	j += matrix[1][1];
	j += matrix[1][2];

	j += matrix[2][0];
	j += matrix[2][1];
	j += matrix[2][2];
}
