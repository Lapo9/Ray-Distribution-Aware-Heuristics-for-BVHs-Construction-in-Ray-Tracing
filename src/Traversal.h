#pragma once

#include <optional>

#include "Utilities.h"
#include "TopLevel.h"

namespace pah {
	namespace traversal {

		std::optional<Triangle> intersectRay(const TopLevel& topLevel, const Ray& ray) {
			const auto& relevantBvhs = topLevel.containedIn(ray.getPosition());

			//TODO resume from here (we have to find out if there is a BVH with the rays almost parallel to the one we have)
		}

	}
}
