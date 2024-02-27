#include "CustomJson.h"

#include "settings.h"

// ======| Bvh |======
void pah::to_json(json& j, const Bvh::Node& node) {
	j["id"] = (unsigned long long int) & node;
	j["aabb"] = node.aabb;
	j["leftChild"] = node.isLeaf() ? 0 : (unsigned long long int) & *node.leftChild;
	j["rightChild"] = node.isLeaf() ? 0 : (unsigned long long int) & *node.rightChild;
	
	for (auto& triangle : node.triangles) {
		j["triangles"] += (unsigned long long int) & *triangle;
	}
}


// ======| Influence areas |======
void pah::to_json(json& j, const InfluenceArea& influenceArea) {
	//we try to cast to the actual type, and then call the proper function with the run-time type; if no cast works, we throw.
	//we have to do like this because C++ does NOT have multiple dispatch, therefore always this version of the function is called (with the static type InfluenceArea)
	try {
		auto& planeInfluenceArea = dynamic_cast<const PlaneInfluenceArea&>(influenceArea);
		j["type"] = "plane";
		to_json(j["planeInfluenceArea"], planeInfluenceArea);
		return;
	} catch (const std::bad_cast&) { /* try next type */ }
	try {
		auto& pointInfluenceArea = dynamic_cast<const PointInfluenceArea&>(influenceArea);
		j["type"] = "point";
		to_json(j["pointInfluenceArea"], pointInfluenceArea);
		return;
	} catch (const std::bad_cast&) { /* try next type */ }
	
	throw std::invalid_argument{ "The run-time type of influenceArea cannot be handled by the to_json function." };
}

void pah::to_json(json& j, const PlaneInfluenceArea& planeInfluenceArea) {
	j["plane"] = planeInfluenceArea.getPlane();
	j["size"] = planeInfluenceArea.getSize();
	j["density"] = planeInfluenceArea.getDensity();
	j["bvhRegion"] = planeInfluenceArea.getBvhRegion();
}

void pah::to_json(json& j, const PointInfluenceArea& pointInfluenceArea) {
	j["type"] = "point";
	j["pov"] = pointInfluenceArea.getPov();
	j["density"] = pointInfluenceArea.getDensity();
	j["bvhRegion"] = pointInfluenceArea.getBvhRegion();
}

// ======| Regions |======
void pah::to_json(json& j, const Region& region) {
	//we try to cast to the actual type, and then call the proper function with the run-time type; if no cast works, we throw.
	//we have to do like this because C++ does NOT have multiple dispatch, therefore always this version of the function is called (with the static type Region)
	try {
		auto& aabb = dynamic_cast<const Aabb&>(region);
		j["type"] = "aabb";
		to_json(j["aabb"], aabb);
		return;
	} catch (const std::bad_cast&) { /* try next type */ }
	try {
		auto& obb = dynamic_cast<const Obb&>(region);
		j["type"] = "obb";
		to_json(j["obb"], obb);
		return;
	} catch (const std::bad_cast&) { /* try next type */ }
	try {
		auto& aabbForObb = dynamic_cast<const AabbForObb&>(region);
		j["type"] = "aabbForObb";
		to_json(j["aabbForObb"], aabbForObb);
		return;
	} catch (const std::bad_cast&) { /* try next type */ }
	try {
		auto& frustum = dynamic_cast<const Frustum&>(region);
		j["type"] = "frustum";
		to_json(j["frustum"], frustum);
		return;
	} catch (const std::bad_cast&) { /* try next type */ }

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
	j["aabb"] = aabbForObb.aabb;
	j["obb"] = aabbForObb.obb;
}

void pah::to_json(json& j, const Frustum& frustum) {
	j["matrix"] = frustum.getViewProjectionMatrix();
	j["vertices"] = frustum.getPoints();
	j["parameters"] = frustum.getViewProjectionMatrixParameters();
}

// ======| Top level |======
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

// ======| Utilities |======
void pah::to_json(json& j, const Triangle& triangle) {
	j["id"] = (unsigned long long int) &triangle;
	j["v0"] = triangle.v0;
	j["v1"] = triangle.v1;
	j["v2"] = triangle.v2;
}

void pah::to_json(json& j, const Plane& plane) {
	j["point"] = plane.getPoint();
	j["normal"] = plane.getNormal();
}

void pah::to_json(json& j, const Pov& pov) {
	j["position"] = pov.position;
	j["direction"] = pov.getDirection();
	j["up"] = pov.getUp();
	j["fovX"] = pov.fovX;
	j["fovY"] = pov.fovY;
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

void pah::to_json(json& j, const TopLevelOctree::NodeTimingInfo& nti) {
	TIME(j["total"] = nti.total.count(););
}

void pah::to_json(json& j, const Bvh::Properties& properties) {
	j["bins"] = properties.bins;
	j["maxLeafCost"] = properties.maxLeafCost;
	j["maxLevels"] = properties.maxLevels;
	j["maxTrianglesPerLeaf"] = properties.maxTrianglesPerLeaf;
}

void pah::to_json(json& j, const TopLevelOctree::Properties& properties) {
	j["conservativeApproach"] = properties.conservativeApproach;
	j["maxLevel"] = properties.maxLevel;
}

void pah::to_json(json& j, const CumulativeRayCasterResults& crcr) {
	j["general"]["raysAmount"] = crcr.raysAmount;
	j["general"]["rayCastersAmount"] = crcr.rayCastersAmount;
	TIME(j["general"]["timeTraversingTotal"] = crcr.timeTraversingTotal.count(););
	TIME(j["general"]["timeTraversingAveragePerRay"] = crcr.timeTraversingAveragePerRay().count(););
	TIME(j["general"]["timeTraversingAveragePerBvh"] = crcr.timeTraversingAveragePerBvh().count(););
	TIME(j["general"]["timeTraversingOnlyBvhsTotal"] = crcr.timeTraversingOnlyBvhsTotal.count(););
	TIME(j["general"]["timeTraversingOnlyBvhsAveragePerRay"] = crcr.timeTraversingOnlyBvhsAveragePerRay().count(););
	TIME(j["general"]["timeTraversingOnlyBvhsAveragePerBvh"] = crcr.timeTraversingOnlyBvhsAveragePerBvh().count(););
	
	j["total"]["bvhsTraversed"]["bvhsTraversedTotal"] = crcr.bvhsTraversedTotal;
	j["total"]["bvhsTraversed"]["bvhsTraversedAveragePerRay"] = crcr.bvhsTraversedAveragePerRay();
	 
	j["total"]["hitMiss"]["hitsTotal"] = crcr.hitsTotal;
	j["total"]["hitMiss"]["hitsPercentage"] = crcr.hitsPercentage();
	j["total"]["hitMiss"]["missesTotal"] = crcr.missesTotal;
	j["total"]["hitMiss"]["missesPercentage"] = crcr.missesPercentage();
	 
	j["total"]["intersectionTests"]["intersectionTestsTotal"] = crcr.intersectionTestsTotal;
	j["total"]["intersectionTests"]["intersectionTestsAveragePerRay"] = crcr.intersectionTestsAveragePerRay();
	j["total"]["intersectionTests"]["intersectionTestsAveragePerBvh"] = crcr.intersectionTestsAveragePerBvh();
	 
	j["total"]["intersectionTestsWithNodes"]["intersectionTestsWithNodesTotal"] = crcr.intersectionTestsWithNodesTotal;
	j["total"]["intersectionTestsWithNodes"]["intersectionTestsWithNodesAveragePerRay"] = crcr.intersectionTestsWithNodesAveragePerRay();
	j["total"]["intersectionTestsWithNodes"]["intersectionTestsWithNodesAveragePerBvh"] = crcr.intersectionTestsWithNodesAveragePerBvh();
	 
	j["total"]["intersectionTestsWithTriangles"]["intersectionTestsWithTrianglesTotal"] = crcr.intersectionTestsWithTrianglesTotal;
	j["total"]["intersectionTestsWithTriangles"]["intersectionTestsWithTrianglesAveragePerRay"] = crcr.intersectionTestsWithTrianglesAveragePerRay();
	j["total"]["intersectionTestsWithTriangles"]["intersectionTestsWithTrianglesAveragePerBvh"] = crcr.intersectionTestsWithTrianglesAveragePerBvh();
	 
	j["total"]["intersectionTestsWhenHit"]["intersectionTestsWhenHitTotal"] = crcr.intersectionTestsWhenHitTotal;
	j["total"]["intersectionTestsWhenHit"]["intersectionTestsWhenHitAveragePerRay"] = crcr.intersectionTestsWhenHitAveragePerRay();
	j["total"]["intersectionTestsWhenHit"]["intersectionTestsWhenHitAveragePerBvh"] = crcr.intersectionTestsWhenHitAveragePerBvh();
	 
	j["total"]["intersectionTestsWhenHitWithNodes"]["intersectionTestsWithNodesWhenHitTotal"] = crcr.intersectionTestsWithNodesWhenHitTotal;
	j["total"]["intersectionTestsWhenHitWithNodes"]["intersectionTestsWithNodesWhenHitAveragePerRay"] = crcr.intersectionTestsWithNodesWhenHitAveragePerRay();
	j["total"]["intersectionTestsWhenHitWithNodes"]["intersectionTestsWithNodesWhenHitAveragePerBvh"] = crcr.intersectionTestsWithNodesWhenHitAveragePerBvh();
	 
	j["total"]["intersectionTestsWhenHitWithTriangles"]["intersectionTestsWithTrianglesWhenHitTotal"] = crcr.intersectionTestsWithTrianglesWhenHitTotal;
	j["total"]["intersectionTestsWhenHitWithTriangles"]["intersectionTestsWithTrianglesWhenHitAveragePerRay"] = crcr.intersectionTestsWithTrianglesWhenHitAveragePerRay();
	j["total"]["intersectionTestsWhenHitWithTriangles"]["intersectionTestsWithTrianglesWhenHitAveragePerBvh"] = crcr.intersectionTestsWithTrianglesWhenHitAveragePerBvh();


	j["fallback"]["general"]["fallbackBvhSearchesTotal"] = crcr.fallbackBvhSearchesTotal;
	j["fallback"]["general"]["fallbackBvhSearchesPercentage"] = crcr.fallbackBvhSearchesPercentage();
	j["fallback"]["general"]["successfulFallbackBvhSearchesPercentage"] = crcr.successfulFallbackBvhSearchesPercentage();
	j["fallback"]["general"]["nonFallbackSearchesTotal"] = crcr.nonFallbackBvhSearches();
	j["fallback"]["general"]["nonFallbackSearchesPercentage"] = crcr.nonFallbackBvhSearchesPercentage();

	j["fallback"]["intersectionTests"]["intersectionTestsNonFallbackTotal"] = crcr.intersectionTestsNonFallbackTotal;
	j["fallback"]["intersectionTests"]["intersectionTestsNonFallbackAveragePerRay"] = crcr.intersectionTestsNonFallbackAveragePerRay();
	j["fallback"]["intersectionTests"]["intersectionTestsNonFallbackAveragePerBvh"] = crcr.intersectionTestsNonFallbackAveragePerBvh();

	j["fallback"]["intersectionTestsWithNodes"]["intersectionTestsWithNodesNonFallbackTotal"] = crcr.intersectionTestsWithNodesNonFallbackTotal;
	j["fallback"]["intersectionTestsWithNodes"]["intersectionTestsWithNodesNonFallbackAveragePerRay"] = crcr.intersectionTestsWithNodesNonFallbackAveragePerRay();
	j["fallback"]["intersectionTestsWithNodes"]["intersectionTestsWithNodesNonFallbackAveragePerBvh"] = crcr.intersectionTestsWithNodesNonFallbackAveragePerBvh();

	j["fallback"]["intersectionTestsWithTriangles"]["intersectionTestsWithTrianglesNonFallbackTotal"] = crcr.intersectionTestsWithTrianglesNonFallbackTotal;
	j["fallback"]["intersectionTestsWithTriangles"]["intersectionTestsWithTrianglesNonFallbackAveragePerRay"] = crcr.intersectionTestsWithTrianglesNonFallbackAveragePerRay();
	j["fallback"]["intersectionTestsWithTriangles"]["intersectionTestsWithTrianglesNonFallbackAveragePerBvh"] = crcr.intersectionTestsWithTrianglesNonFallbackAveragePerBvh();

	j["fallback"]["intersectionTestsWhenHit"]["intersectionTestsWhenHitNonFallbackTotal"] = crcr.intersectionTestsWhenHitNonFallbackTotal;
	j["fallback"]["intersectionTestsWhenHit"]["intersectionTestsWhenHitNonFallbackAveragePerRay"] = crcr.intersectionTestsWhenHitNonFallbackAveragePerRay();
	j["fallback"]["intersectionTestsWhenHit"]["intersectionTestsWhenHitNonFallbackAveragePerBvh"] = crcr.intersectionTestsWhenHitNonFallbackAveragePerBvh();

	j["fallback"]["intersectionTestsWhenHitWithNodes"]["intersectionTestsWithNodesWhenHitNonFallbackTotal"] = crcr.intersectionTestsWithNodesWhenHitNonFallbackTotal;
	j["fallback"]["intersectionTestsWhenHitWithNodes"]["intersectionTestsWithNodesWhenHitNonFallbackAveragePerRay"] = crcr.intersectionTestsWithNodesWhenHitNonFallbackAveragePerRay();
	j["fallback"]["intersectionTestsWhenHitWithNodes"]["intersectionTestsWithNodesWhenHitNonFallbackAveragePerBvh"] = crcr.intersectionTestsWithNodesWhenHitNonFallbackAveragePerBvh();

	j["fallback"]["intersectionTestsWhenHitWithTriangles"]["intersectionTestsWithTrianglesWhenHitNonFallbackTotal"] = crcr.intersectionTestsWithTrianglesWhenHitNonFallbackTotal;
	j["fallback"]["intersectionTestsWhenHitWithTriangles"]["intersectionTestsWithTrianglesWhenHitNonFallbackAveragePerRay"] = crcr.intersectionTestsWithTrianglesWhenHitNonFallbackAveragePerRay();
	j["fallback"]["intersectionTestsWhenHitWithTriangles"]["intersectionTestsWithTrianglesWhenHitNonFallbackAveragePerBvh"] = crcr.intersectionTestsWithTrianglesWhenHitNonFallbackAveragePerBvh();
}

void pah::projection::to_json(json& j, const ProjectionMatrixParameters& params) {
	j["n"] = params.n;
	j["f"] = params.f;
	j["b"] = params.b;
	j["t"] = params.t;
	j["l"] = params.l;
	j["r"] = params.r;
	j["fovX"] = params.fovX;
	j["fovY"] = params.fovY;
	j["ratio"] = params.ratio;
}

void glm::to_json(json& j, const pah::Vector4& vector) {
	j += vector.x;
	j += vector.y;
	j += vector.z;
	j += vector.w;
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
	j += matrix[0];
	j += matrix[1];
	j += matrix[2];
	j += matrix[3];
}

void glm::to_json(json& j, const pah::Matrix3& matrix) {
	j += matrix[0];
	j += matrix[1];
	j += matrix[2];
}

