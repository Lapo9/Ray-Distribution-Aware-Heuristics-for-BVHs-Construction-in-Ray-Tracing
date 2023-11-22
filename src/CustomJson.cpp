#include "CustomJson.h"

void pah::to_json(json& j, const Bvh::Node& node) {
	j["id"] = (unsigned long long int) & node;
	j["aabb"] = node.aabb;
	j["leftChild"] = (unsigned long long int) & node.leftChild;
	j["rightChild"] = (unsigned long long int) & node.rightChild;
	j["trianglesAmount"] = node.triangles.size();
}

void pah::to_json(json& j, const Aabb& aabb) {
	j["min"] = aabb.min;
	j["max"] = aabb.max;
}

void pah::to_json(json& j, const Triangle& triangle) {
	j["v1"] = triangle.v1;
	j["v2"] = triangle.v2;
	j["v3"] = triangle.v3;
}

void glm::to_json(json& j, const pah::Vector3& vector) {
	j["coords"] = { vector.x, vector.y, vector.z };
}

void glm::to_json(json& j, const pah::Vector2& vector) {
	j["coords"] = { vector.x, vector.y};
}
