#include "pch.h"

#include "../../ProjectedAreaHeuristic/src/Utilities.h"
#include "../../ProjectedAreaHeuristic/src/Regions.h"
#include "../../ProjectedAreaHeuristic/src/Projections.h"


namespace perspective {
	TEST(Projection, Aabbs) {
		using namespace pah;

		Aabb aabb1{ {-13.12453, -0.035194, -10.31048}, {9.326398, 9.284466, 12.07444} };
		Aabb aabb2{ {-13.12453,  0.003557, -10.24038}, {6.403256, 9.284466, 12.07444} };

		Pov pov1{ {18,2.7,0}, {-1,0,0}, 70, 35 };

		auto contourPoints1 = projection::perspective::findContourPoints(aabb1, pov1.position);
		auto projContPoints1 = projection::perspective::projectPoints(contourPoints1, pov1);
		auto area1 = projection::perspective::computeProjectedArea(projContPoints1);
		auto area1Auto = projection::perspective::computeProjectedArea(aabb1, pov1);
		EXPECT_EQ(area1, area1Auto);
		
		auto contourPoints2 = projection::perspective::findContourPoints(aabb2, pov1.position);
		auto projContPoints2 = projection::perspective::projectPoints(contourPoints2, pov1);
		auto area2 = projection::perspective::computeProjectedArea(projContPoints2);
		auto area2Auto = projection::perspective::computeProjectedArea(aabb2, pov1);
		EXPECT_EQ(area2, area2Auto);
	}
}
