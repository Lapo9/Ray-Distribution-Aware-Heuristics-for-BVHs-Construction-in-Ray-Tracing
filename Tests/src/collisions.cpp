#include "pch.h"

#include "../../ProjectedAreaHeuristic/src/Utilities.h"
#include "../../ProjectedAreaHeuristic/src/Regions.h"


namespace collisions {
	// Standard Ray-Triangle scenario
	TEST(RayTriangle, StandardHit) {
		using namespace pah;

		Triangle t1{ Vector3{1,1,1}, Vector3{2,2,2}, Vector3{1,2,3} };
		Ray r1{ Vector3{2,-1,0}, Vector3{-0.8f,3.7f,2.9f} };
		auto res1 = collisionDetection::areColliding(r1, t1);
		EXPECT_TRUE(res1.hit) << "Ray r1 should be colliding with Triangle t1.";
		EXPECT_NEAR(res1.distance, 3.599f, TOLERANCE) << "Collision distance of Ray r1 and Triangle t1 should be 3.599.";

		Triangle t2{ Vector3{2,1,-2}, Vector3{-1,2,0}, Vector3{1,1,3} };
		Ray r2{ Vector3{-2,1,1}, Vector3{3.2f,0.9f,-1.4f} };
		auto res2 = collisionDetection::areColliding(r2, t2);
		EXPECT_TRUE(res2.hit) << "Ray r2 should be colliding with Triangle t2.";
		EXPECT_NEAR(res2.distance, 2.331f, TOLERANCE) << "Collision distance of Ray r2 and Triangle t2 should be 2.331.";		
		
		auto res3 = collisionDetection::areColliding(r2, t1);
		EXPECT_FALSE(res3.hit) << "Ray r2 should not be colliding with Triangle t1.";
	}

	// The line of the Ray hits the Triangle, but the Ray goes in the opposite direction
	TEST(RayTriangle, HitBehind) {
		using namespace pah;

		Triangle t1{ Vector3{1,1,1}, Vector3{2,2,2}, Vector3{1,2,3} };
		Ray r1{ Vector3{2,-1,0}, Vector3{0.8f,-3.7f,-2.9f} };
		auto res1 = collisionDetection::areColliding(r1, t1);
		EXPECT_FALSE(res1.hit) << "Ray r1 should not be colliding with Triangle t1.";
	}

	// The Ray and the planw of the Triangle are parallel
	TEST(RayTriangle, ParallelRay) {
		using namespace pah;

		Triangle t1{ Vector3{1,1,1}, Vector3{2,2,2}, Vector3{1,2,3} };
		Ray r1{ Vector3{2,-1,0}, Vector3{1,1,1} };
		auto res1 = collisionDetection::areColliding(r1, t1);
		EXPECT_FALSE(res1.hit) << "Ray r1 should not be colliding with Triangle t1.";

		Triangle t2{ Vector3{1,1,1}, Vector3{2,2,1}, Vector3{1,2,1} };
		Ray r2{ Vector3{1,0,1}, Vector3{0,1,0} };
		auto res2 = collisionDetection::areColliding(r2, t2);
		EXPECT_FALSE(res2.hit) << "Ray r2 should not be colliding with Triangle t2.";
	}

	// Standard Ray-Aabb scenario
	TEST(RayAabb, StandardHit) {
		using namespace pah;
		Aabb aabb1{ Vector3{2,2,-2}, Vector3{4,4,-1} };
		Ray r1{ Vector3{-2,1,1}, Vector3{3.2f,0.9f,-1.4f} };
		auto res1 = collisionDetection::areColliding(r1, aabb1);
		EXPECT_TRUE(res1.hit) << "Ray r1 should be colliding with Aabb aabb1.";
		EXPECT_NEAR(res1.distance, 5.153f, TOLERANCE) << "Collision distance of Ray r1 and Aabb aabb1 should be 5.153.";

		Ray r2{ Vector3{-2,1,1}, Vector3{-3.2f,-0.9f,1.4f} };
		auto res2 = collisionDetection::areColliding(r2, aabb1);
		EXPECT_FALSE(res2.hit) << "Ray r2 should not be colliding with Aabb aabb1.";
	}

	// The Ray is parallel to at least one of the planes of the Aabb (i.e. at least one of the components of the Ray is 0, which causes a divison by 0 which should be handled by IEEE floating points rules)
	TEST(RayAabb, SlabAtInfinite) {
		using namespace pah;
		Aabb aabb1{ Vector3{2,2,-2}, Vector3{4,4,-1} };
		Ray r1{ Vector3{3,3,1}, Vector3{0,0,-1} };
		auto res1 = collisionDetection::areColliding(r1, aabb1);
		EXPECT_TRUE(res1.hit) << "Ray r1 should be colliding with Aabb aabb1.";
		EXPECT_NEAR(res1.distance, 2, TOLERANCE) << "Collision distance of Ray r1 and Aabb aabb1 should be 2.";

		Ray r2{ Vector3{10,10,10}, Vector3{0,0,-1} };
		auto res2 = collisionDetection::areColliding(r2, aabb1);
		EXPECT_FALSE(res2.hit) << "Ray r2 should not be colliding with Aabb aabb1.";
	}

	// The Ray origin is inside the Aabb
	TEST(RayAabb, OriginInsideAabb) {
		using namespace pah;
		Aabb aabb1{ Vector3{2,2,-2}, Vector3{4,4,-1} };
		Ray r1{ Vector3{3,3.5f,-1.5}, Vector3{1,2,3} };
		auto res1 = collisionDetection::areColliding(r1, aabb1);
		EXPECT_TRUE(res1.hit) << "Ray r1 should be colliding with Aabb aabb1.";
		EXPECT_NEAR(res1.distance, 0.624f, TOLERANCE) << "Collision distance of Ray r1 and Aabb aabb1 should be 0.624.";
	}
}
