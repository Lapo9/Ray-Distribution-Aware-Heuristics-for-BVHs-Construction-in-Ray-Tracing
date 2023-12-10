#pragma once

#include <vector>

#include "Utilities.h"

namespace pah {
	
	class BvhRegion {
	public:
		virtual bool isInside(const Vector3& point) const = 0;
	};


	class ObbBvhRegion : public BvhRegion {
	public:
		ObbBvhRegion(const Vector3& center, const Vector3& halfSize, const Vector3& forward);

		bool isInside(const Vector3& point) const override;
		const Obb& getObb() const;

	private:
		bool isInsideObb(const Vector3& point) const;
		bool isInsideAabb(const Vector3& point) const;

		Obb obb;
		Aabb enclosingAabb;
	};
}