#include "pch.h"

#include "../../ProjectedAreaHeuristic/src/Utilities.h"
#include "../../ProjectedAreaHeuristic/src/Regions.h"
#include "../../ProjectedAreaHeuristic/src/InfluenceArea.h"
#include "../../ProjectedAreaHeuristic/src/InfluenceArea.cpp"
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

	TEST(Overlapping, AabbsOverlapping) {
		using namespace pah;

		Aabb aabb1{ {-1,-1,-1},{1,1,1} };
		Aabb aabb2{ {2,-1,-1},{3,1,1} };

		Plane facingRight{ {-2,0,0},{1,0,0} };
		Plane facingRightFwd{ {-2,0,-2},{1,0,1} };

		auto contourPoints1Right = projection::orthographic::findContourPoints(aabb1, facingRight.getNormal());
		auto contourPoints2Right = projection::orthographic::findContourPoints(aabb2, facingRight.getNormal());
		ConvexHull2d contourPoints1RightProj{ projection::orthographic::projectPoints(contourPoints1Right, facingRight) };
		ConvexHull2d contourPoints2RightProj{ projection::orthographic::projectPoints(contourPoints2Right, facingRight) };

		auto contourPoints1RightFwd = projection::orthographic::findContourPoints(aabb1, facingRightFwd.getNormal());
		auto contourPoints2RightFwd = projection::orthographic::findContourPoints(aabb2, facingRightFwd.getNormal());
		ConvexHull2d contourPoints1RightFwdProj{ projection::orthographic::projectPoints(contourPoints1RightFwd, facingRightFwd) };
		ConvexHull2d contourPoints2RightFwdProj{ projection::orthographic::projectPoints(contourPoints2RightFwd, facingRightFwd) };

		float overlappingAreaRight = overlappingArea(contourPoints1RightProj, contourPoints2RightProj);
		float areaRight1 = contourPoints1RightProj.computeArea();
		float areaRight2 = contourPoints2RightProj.computeArea();

		float overlappingAreaRightFwd = overlappingArea(contourPoints1RightFwdProj, contourPoints2RightFwdProj);
		float areaRightFwd1 = contourPoints1RightFwdProj.computeArea();
		float areaRightFwd2 = contourPoints2RightFwdProj.computeArea();

		ConvexHull2d internalHull{ {{-1,-1},{1,-1},{1,1},{-1,1}} };
		ConvexHull2d externalHull{ {{-2,-2},{2,-2},{2,2},{-2,2}} };
		float overlappingAreaInternal = overlappingArea(internalHull, externalHull);

		int a = 1;
	}


	TEST(OrthographicArea, OrthographicAreaFast) {
		using namespace pah;

		Aabb aabb1{ {-1,-1,-1},{1,1,1} };
		Plane facingRightFwd{ {-2,0,-2},{1,0,1} };

		float area = projection::orthographic::computeProjectedArea(aabb1, facingRightFwd);
	}


	TEST(ProjectionPlaneArea, VerifyProjectionPlaneArea) {
		using namespace pah;

		PointInfluenceArea pointInfluenceArea{ Pov{{0,0,0}, {-1,0,-1}, 90, 60}, 1000, 1, 100 };
		Aabb aabb{ {2,-1.16,-2}, {3,1.16,2} };
		float area = pointInfluenceArea.getProjectionPlaneArea();
		float aabbArea = pointInfluenceArea.getProjectedArea(aabb);
		auto aabbHull = pointInfluenceArea.getProjectedHull(aabb);

		Vector3 p{ -1,0,-1 };
		Vector2 pp = projection::orthographic::projectPoint(p, Plane{ {0,0,0}, {-1,0,-1}, 1,1 });

		int a = 1;
	}
}
