#include "CustomJson.h"

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

void pah::to_json(json& j, const Triangle& triangle) {
	j["id"] = (unsigned long long int) &triangle;
	j["v1"] = triangle.v1;
	j["v2"] = triangle.v2;
	j["v3"] = triangle.v3;
}

void pah::to_json(json& j, const InfluenceArea& influenceArea) {
	//we try to cast to the actual type, and then call the proper function with the run-time type; if no cast works, we throw.
	//we have to do like this because C++ does NOT have multiple dispatch, therefore always this version of the function is called (with the static type InfluenceArea.
	try {
		auto& planeInfluenceArea = dynamic_cast<const PlaneInfluenceArea&>(influenceArea);
		to_json(j, planeInfluenceArea);
		return;
	} catch (const std::bad_cast&){}
	
	throw std::invalid_argument{ "The run-time type of influenceArea cannot be handled by the to_json function." };
}

void pah::to_json(json& j, const PlaneInfluenceArea& planeInfluenceArea) {
	j["type"] = "plane";
	j["plane"] = planeInfluenceArea.getPlane();
	j["size"] = planeInfluenceArea.getSize();
	j["density"] = planeInfluenceArea.getDensity();
}

void pah::to_json(json& j, const Plane& plane) {
	j["point"] = plane.getPoint();
	j["normal"] = plane.getNormal();
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
