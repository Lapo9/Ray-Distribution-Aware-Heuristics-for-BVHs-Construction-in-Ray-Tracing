#pragma once

#include "../libs/json.hpp"
#include "Bvh.h"
#include "Utilities.h"
#include "InfluenceArea.h"
#include "Regions.h"


namespace glm{
	using json = nlohmann::json;

	void to_json(json& j, const pah::Vector3& vector);
	void to_json(json& j, const pah::Vector2& vector);
}


namespace pah {
	using json = nlohmann::json;

	void to_json(json& j, const Bvh::Node& node);
	void to_json(json& j, const Aabb& aabb);
	void to_json(json& j, const Obb& obb);
	void to_json(json& j, const Triangle& triangle);
	void to_json(json& j, const InfluenceArea& influenceArea);
	void to_json(json& j, const PlaneInfluenceArea& planeInfluenceArea);
	void to_json(json& j, const Region& region);
	void to_json(json& j, const AabbForObb& aabbForObb);
	void to_json(json& j, const Plane& plane);
	void to_json(json& j, const Bvh::NodeTimingInfo& nti);
}