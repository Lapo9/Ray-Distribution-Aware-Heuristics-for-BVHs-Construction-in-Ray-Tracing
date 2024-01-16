#include "RayCaster.h"

using namespace std;
using namespace pah;

// ======| CumulativeRayCasterResults |======

CumulativeRayCasterResults& pah::CumulativeRayCasterResults::operator+=(const CumulativeRayCasterResults& rhs) {
	raysAmount += rhs.raysAmount;
	rayCastersAmount += rhs.rayCastersAmount;
	totalBvhsTraversed += rhs.totalBvhsTraversed;
	totalHits += rhs.totalHits;
	totalMisses += rhs.totalMisses;
	totalIntersectionTests += rhs.totalIntersectionTests;
	totalIntersectionTestsWithNodes += rhs.totalIntersectionTestsWithNodes;
	totalIntersectionTestsWithTriangles += rhs.totalIntersectionTestsWithTriangles;
	totalIntersectionTestsWhenHit += rhs.totalIntersectionTestsWhenHit;
	totalIntersectionTestsWithNodesWhenHit += rhs.totalIntersectionTestsWithNodesWhenHit;
	totalIntersectionTestsWithTrianglesWhenHit += rhs.totalIntersectionTestsWithTrianglesWhenHit;
	totalFallbackBvhSearches += rhs.totalFallbackBvhSearches;
	TIME(totalTimeTraversing += rhs.totalTimeTraversing;);

	return *this;
}

CumulativeRayCasterResults pah::operator+(CumulativeRayCasterResults lhs, const CumulativeRayCasterResults& rhs) {
	return lhs += rhs;
}

CumulativeRayCasterResults& pah::CumulativeRayCasterResults::operator+=(const RayCasterResults& rhs) {
	raysAmount += rhs.raysAmount;
	rayCastersAmount++;
	totalBvhsTraversed += rhs.totalBvhsTraversed;
	totalHits += rhs.totalHits;
	totalMisses += rhs.totalMisses;
	totalIntersectionTests += rhs.totalIntersectionTests;
	totalIntersectionTestsWithNodes += rhs.totalIntersectionTestsWithNodes;
	totalIntersectionTestsWithTriangles += rhs.totalIntersectionTestsWithTriangles;
	totalIntersectionTestsWhenHit += rhs.totalIntersectionTestsWhenHit;
	totalIntersectionTestsWithNodesWhenHit += rhs.totalIntersectionTestsWithNodesWhenHit;
	totalIntersectionTestsWithTrianglesWhenHit += rhs.totalIntersectionTestsWithTrianglesWhenHit;
	totalFallbackBvhSearches += rhs.totalFallbackBvhSearches;
	TIME(totalTimeTraversing += rhs.totalTimeTraversing;);

	return *this;
}

CumulativeRayCasterResults pah::operator+(CumulativeRayCasterResults lhs, const RayCasterResults& rhs) {
	return lhs += rhs;
}

float pah::CumulativeRayCasterResults::averageBvhsTraversedPerRay() const {
	return (float)totalBvhsTraversed / raysAmount;
}

float pah::CumulativeRayCasterResults::hitsPercentage() const {
	return (float)totalHits / raysAmount;
}

float pah::CumulativeRayCasterResults::missesPercentage() const {
	return (float)totalMisses / raysAmount;
}

float pah::CumulativeRayCasterResults::averageIntersectionTestsPerRay() const {
	return (float)totalIntersectionTests / raysAmount;
}

float pah::CumulativeRayCasterResults::averageIntersectionTestsWithNodesPerRay() const {
	return (float)totalIntersectionTestsWithNodes / raysAmount;
}

float pah::CumulativeRayCasterResults::averageIntersectionTestsWithTrianglesPerRay() const {
	return (float)totalIntersectionTestsWithTriangles / raysAmount;
}

float pah::CumulativeRayCasterResults::averageIntersectionTestsWhenHitPerRay() const {
	return (float)totalIntersectionTestsWhenHit / raysAmount;
}

float pah::CumulativeRayCasterResults::averageIntersectionTestsWithNodesWhenHitPerRay() const {
	return (float)totalIntersectionTestsWithNodesWhenHit / raysAmount;
}

float pah::CumulativeRayCasterResults::averageIntersectionTestsWithTrianglesWhenHitPerRay() const {
	return (float)totalIntersectionTestsWithTrianglesWhenHit / raysAmount;
}

float pah::CumulativeRayCasterResults::fallbackBvhSearchesPercentage() const {
	return totalFallbackBvhSearches / (float)raysAmount;
}

float pah::CumulativeRayCasterResults::successfulFallbackBvhSearchesPercentage() const {
	return 1.0f - (float)totalMisses / totalFallbackBvhSearches; //remember that all misses come from a BVH search
}

TIME(DurationMs pah::CumulativeRayCasterResults::averageTimeTraversingPerRay() const {
	return totalTimeTraversing / (float)raysAmount;
});


// ======| RayCasterResults |======

RayCasterResults& pah::RayCasterResults::operator+=(const TopLevel::TraversalResults& rhs) {
	raysAmount++;
	totalBvhsTraversed += rhs.bvhsTraversed;
	totalHits += rhs.hit();
	totalMisses += !rhs.hit();
	totalIntersectionTests += rhs.intersectionTestsTotal;
	totalIntersectionTestsWithNodes += rhs.intersectionTestsWithNodes;
	totalIntersectionTestsWithTriangles += rhs.intersectionTestsWithTriangles;
	totalFallbackBvhSearches += rhs.fallbackBvhSearch;
	TIME(totalTimeTraversing += rhs.traversalTime;);
	if (rhs.hit()) {
		totalIntersectionTestsWhenHit += rhs.intersectionTestsWhenHit;
		totalIntersectionTestsWithNodesWhenHit += rhs.intersectionTestsWithNodesWhenHit;
		totalIntersectionTestsWithTrianglesWhenHit += rhs.intersectionTestsWithTrianglesWhenHit;
	}

	return *this;
}

RayCasterResults& pah::RayCasterResults::operator+=(const Bvh::TraversalResults& rhs) {
	raysAmount++;
	totalBvhsTraversed++;
	totalHits += rhs.hit();
	totalMisses += !rhs.hit();
	totalIntersectionTests += rhs.intersectionTestsTotal;
	totalIntersectionTestsWithNodes += rhs.intersectionTestsWithNodes;
	totalIntersectionTestsWithTriangles += rhs.intersectionTestsWithTriangles;
	TIME(totalTimeTraversing += rhs.traversalTime;);
	if (rhs.hit()) {
		totalIntersectionTestsWhenHit = rhs.intersectionTestsTotal;
		totalIntersectionTestsWithNodesWhenHit = rhs.intersectionTestsWithNodes;
		totalIntersectionTestsWithTrianglesWhenHit = rhs.intersectionTestsWithTriangles;
	}
	return *this;
}

CumulativeRayCasterResults pah::operator+(const RayCasterResults& lhs, const RayCasterResults& rhs) {
	CumulativeRayCasterResults crcr{};
	crcr.raysAmount = lhs.raysAmount + rhs.raysAmount;
	crcr.rayCastersAmount = 2;
	crcr.totalBvhsTraversed = lhs.totalBvhsTraversed + rhs.totalBvhsTraversed;
	crcr.totalHits = lhs.totalHits + rhs.totalHits;
	crcr.totalMisses = lhs.totalMisses + rhs.totalMisses;
	crcr.totalIntersectionTests = lhs.totalIntersectionTests + rhs.totalIntersectionTests;
	crcr.totalIntersectionTestsWithNodes = lhs.totalIntersectionTestsWithNodes + rhs.totalIntersectionTestsWithNodes;
	crcr.totalIntersectionTestsWithTriangles = lhs.totalIntersectionTestsWithTriangles + rhs.totalIntersectionTestsWithTriangles;
	crcr.totalIntersectionTestsWhenHit = lhs.totalIntersectionTestsWhenHit + rhs.totalIntersectionTestsWhenHit;
	crcr.totalIntersectionTestsWithNodesWhenHit = lhs.totalIntersectionTestsWithNodesWhenHit + rhs.totalIntersectionTestsWithNodesWhenHit;
	crcr.totalIntersectionTestsWithTrianglesWhenHit = lhs.totalIntersectionTestsWithTrianglesWhenHit + rhs.totalIntersectionTestsWithTrianglesWhenHit;
	TIME(crcr.totalTimeTraversing = lhs.totalTimeTraversing + rhs.totalTimeTraversing;);

	return crcr;
}

float pah::RayCasterResults::averageBvhsTraversedPerRay() const {
	return (float)totalBvhsTraversed / raysAmount;
}

float pah::RayCasterResults::hitsPercentage() const {
	return (float)totalHits / raysAmount;
}

float pah::RayCasterResults::missesPercentage() const {
	return (float)totalMisses / raysAmount;
}

float pah::RayCasterResults::averageIntersectionTestsPerRay() const {
	return (float)totalIntersectionTests / raysAmount;
}

float pah::RayCasterResults::averageIntersectionTestsWithNodesPerRay() const {
	return (float)totalIntersectionTestsWithNodes / raysAmount;
}

float pah::RayCasterResults::averageIntersectionTestsWithTrianglesPerRay() const {
	return (float)totalIntersectionTestsWithTriangles / raysAmount;
}

float pah::RayCasterResults::averageIntersectionTestsWhenHitPerRay() const {
	return (float)totalIntersectionTestsWhenHit / raysAmount;
}

float pah::RayCasterResults::averageIntersectionTestsWithNodesWhenHitPerRay() const {
	return (float)totalIntersectionTestsWithNodesWhenHit / raysAmount;
}

float pah::RayCasterResults::averageIntersectionTestsWithTrianglesWhenHitPerRay() const {
	return (float)totalIntersectionTestsWithTrianglesWhenHit / raysAmount;
}

float pah::RayCasterResults::fallbackBvhSearchesPercentage() const {
	return totalFallbackBvhSearches / (float)raysAmount;
}

float pah::RayCasterResults::successfulFallbackBvhSearchesPercentage() const {
	return 1.0f - totalMisses / totalFallbackBvhSearches; //remember that all misses come from a BVH search
}

TIME(DurationMs pah::RayCasterResults::averageTimeTraversingPerRay() const {
	return totalTimeTraversing / (float)raysAmount;
})

