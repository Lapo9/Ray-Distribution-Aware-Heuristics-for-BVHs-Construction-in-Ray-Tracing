#pragma once

#include "../libs/json.hpp"
#include "Bvh.h"
#include "Utilities.h"
#include "BvhAnalyzer.h"


namespace glm{
	using json = nlohmann::json;

	void to_json(json& j, const pah::Vector3& vector);
	void to_json(json& j, const pah::Vector2& vector);
}


namespace pah {
	using json = nlohmann::json;

	void to_json(json& j, const Bvh::Node& node);
	void to_json(json& j, const Aabb& aabb);
	void to_json(json& j, const Triangle& triangle);
}