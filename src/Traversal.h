#pragma once

#include <optional>

#include "Utilities.h"
#include "TopLevel.h"

namespace pah {
	namespace traversal {

		std::optional<Triangle> intersectRay(const TopLevel& topLevel, const Ray& ray) {
			const auto& relevantBvhs = topLevel.containedIn(ray.getPosition());

			for (const auto& bvh : relevantBvhs) {
				if (bvh->getInfluenceArea().isDirectionAffine(ray.getDirection(), TOLERANCE)) { //TODO tolerance here should be a bigger value (we have to tune it)

				}
			}
		}

	}
}
