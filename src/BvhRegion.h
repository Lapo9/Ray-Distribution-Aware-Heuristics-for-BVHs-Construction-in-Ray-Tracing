#pragma once

#include <vector>

#include "Utilities.h"

namespace pah {
	
	class BvhRegion {
	public:
		virtual bool isInside(const Vector3& point) const = 0;
		virtual bool isInside(const Triangle& triangle) const = 0;
		virtual void fill(const vector<Triangle>& scene) = 0;
	};


	class BoxBvhRegion : public BvhRegion {
		
	};
}