#include "pch.h"

#include "../../ProjectedAreaHeuristic/src/Utilities.h"
#include "../../ProjectedAreaHeuristic/src/Regions.h"
#include "../../ProjectedAreaHeuristic/src/Projections.h"


namespace perspective {
	TEST(Projection, Aabbs) {
		using namespace pah;

		Aabb aabb1{ {-1.367188, -0.984375, -0.851562}, {1.367188, 0.984375, 0.851562} };
		Aabb aabb2{ {0,  -0.984375, -0.851562}, {1.367188, 0.984375, 0.851562} };

		Pov pov1{ {0,0,2}, aabb1.center() - Vector3{0,0,2}, 90, 90 };
		Pov pov2{ {0,0,2}, aabb2.center() - Vector3{0,0,2}, 90, 90 };

		auto contourPoints1 = projection::perspective::findContourPoints(aabb1, pov1.position);
		auto projContPoints1 = projection::perspective::projectPoints(contourPoints1, pov1);
		auto area1 = projection::perspective::computeProjectedArea(projContPoints1);
		auto area1Auto = projection::perspective::computeProjectedArea(aabb1, pov1);
		EXPECT_EQ(area1, area1Auto);
		
		auto contourPoints2 = projection::perspective::findContourPoints(aabb2, pov2.position);
		auto projContPoints2 = projection::perspective::projectPoints(contourPoints2, pov2);
		auto area2 = projection::perspective::computeProjectedArea(projContPoints2);
		auto area2Auto = projection::perspective::computeProjectedArea(aabb2, pov2);
		EXPECT_EQ(area2, area2Auto);
	}
}
