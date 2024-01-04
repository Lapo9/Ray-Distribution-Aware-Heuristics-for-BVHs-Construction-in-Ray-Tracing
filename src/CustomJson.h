#pragma once

#include "../libs/json.hpp"
#include "Bvh.h"
#include "Utilities.h"
#include "InfluenceArea.h"
#include "Regions.h"
#include "TopLevel.h"
#include "Projections.h"


namespace glm{
	using json = nlohmann::json;

	void to_json(json& j, const pah::Vector4&);
	void to_json(json& j, const pah::Vector3&);
	void to_json(json& j, const pah::Vector2&);
	void to_json(json& j, const pah::Matrix3&);
	void to_json(json& j, const pah::Matrix4&);
}


namespace pah {
	using json = nlohmann::json;

	void to_json(json& j, const Bvh::Node&);
	void to_json(json& j, const InfluenceArea&);
	void to_json(json& j, const PlaneInfluenceArea&);
	void to_json(json& j, const PointInfluenceArea&);
	void to_json(json& j, const Region&);
	void to_json(json& j, const Aabb&);
	void to_json(json& j, const Obb&);
	void to_json(json& j, const AabbForObb&);
	void to_json(json& j, const Frustum&);
	void to_json(json& j, const TopLevelOctree::Node&);
	void to_json(json& j, const Plane&);
	void to_json(json& j, const Pov&);
	void to_json(json& j, const Triangle&);
	void to_json(json& j, const Bvh::NodeTimingInfo&);

	namespace projection {
		void to_json(json& j, const ProjectionMatrixParameters&);
	}
}