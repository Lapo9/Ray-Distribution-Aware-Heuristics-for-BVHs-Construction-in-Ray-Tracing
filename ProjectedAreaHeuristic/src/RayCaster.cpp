#include "RayCaster.h"

using namespace std;
using namespace pah;

// ======| CumulativeRayCasterResults |======

CumulativeRayCasterResults& pah::CumulativeRayCasterResults::operator+=(const CumulativeRayCasterResults& rhs) {
	raysAmount += rhs.raysAmount;
	rayCastersAmount += rhs.rayCastersAmount;
	bvhsTraversedTotal += rhs.bvhsTraversedTotal;
	hitsTotal += rhs.hitsTotal;
	missesTotal += rhs.missesTotal;
	intersectionTestsTotal += rhs.intersectionTestsTotal;
	intersectionTestsWithNodesTotal += rhs.intersectionTestsWithNodesTotal;
	intersectionTestsWithTrianglesTotal += rhs.intersectionTestsWithTrianglesTotal;
	intersectionTestsWhenHitTotal += rhs.intersectionTestsWhenHitTotal;
	intersectionTestsWithNodesWhenHitTotal += rhs.intersectionTestsWithNodesWhenHitTotal;
	intersectionTestsWithTrianglesWhenHitTotal += rhs.intersectionTestsWithTrianglesWhenHitTotal;
	fallbackBvhSearchesTotal += rhs.fallbackBvhSearchesTotal;
	
	intersectionTestsNonFallbackTotal += rhs.intersectionTestsNonFallbackTotal;
	intersectionTestsWithNodesNonFallbackTotal += rhs.intersectionTestsWithNodesNonFallbackTotal;
	intersectionTestsWithTrianglesNonFallbackTotal += rhs.intersectionTestsWithTrianglesNonFallbackTotal;
	intersectionTestsWhenHitNonFallbackTotal += rhs.intersectionTestsWhenHitNonFallbackTotal;
	intersectionTestsWithNodesWhenHitNonFallbackTotal += rhs.intersectionTestsWithNodesWhenHitNonFallbackTotal;
	intersectionTestsWithTrianglesWhenHitNonFallbackTotal += rhs.intersectionTestsWithTrianglesWhenHitNonFallbackTotal; 
	TIME(timeTraversingTotal += rhs.timeTraversingTotal;);

	return *this;
}

CumulativeRayCasterResults pah::operator+(CumulativeRayCasterResults lhs, const CumulativeRayCasterResults& rhs) {
	return lhs += rhs;
}

CumulativeRayCasterResults& pah::CumulativeRayCasterResults::operator+=(const RayCasterResults& rhs) {
	raysAmount += rhs.raysAmount;
	rayCastersAmount++;
	bvhsTraversedTotal += rhs.bvhsTraversedTotal;
	hitsTotal += rhs.hitsTotal;
	missesTotal += rhs.missesTotal;
	intersectionTestsTotal += rhs.intersectionTestsTotal;
	intersectionTestsWithNodesTotal += rhs.intersectionTestsWithNodesTotal;
	intersectionTestsWithTrianglesTotal += rhs.intersectionTestsWithTrianglesTotal;
	intersectionTestsWhenHitTotal += rhs.intersectionTestsWhenHitTotal;
	intersectionTestsWithNodesWhenHitTotal += rhs.intersectionTestsWithNodesWhenHitTotal;
	intersectionTestsWithTrianglesWhenHitTotal += rhs.intersectionTestsWithTrianglesWhenHitTotal;
	fallbackBvhSearchesTotal += rhs.fallbackBvhSearchesTotal;

	intersectionTestsNonFallbackTotal += rhs.intersectionTestsNonFallbackTotal;
	intersectionTestsWithNodesNonFallbackTotal += rhs.intersectionTestsWithNodesNonFallbackTotal;
	intersectionTestsWithTrianglesNonFallbackTotal += rhs.intersectionTestsWithTrianglesNonFallbackTotal;
	intersectionTestsWhenHitNonFallbackTotal += rhs.intersectionTestsWhenHitNonFallbackTotal;
	intersectionTestsWithNodesWhenHitNonFallbackTotal += rhs.intersectionTestsWithNodesWhenHitNonFallbackTotal;
	intersectionTestsWithTrianglesWhenHitNonFallbackTotal += rhs.intersectionTestsWithTrianglesWhenHitNonFallbackTotal; 
	TIME(timeTraversingTotal += rhs.timeTraversingTotal;);

	return *this;
}

CumulativeRayCasterResults pah::operator+(CumulativeRayCasterResults lhs, const RayCasterResults& rhs) {
	return lhs += rhs;
}


// ======| RayCasterResults |======

RayCasterResults& pah::RayCasterResults::operator+=(const TopLevel::TraversalResults& rhs) {
	raysAmount++;
	bvhsTraversedTotal += rhs.bvhsTraversed;
	hitsTotal += rhs.hit();
	missesTotal += !rhs.hit();
	intersectionTestsTotal += rhs.intersectionTestsTotal;
	intersectionTestsWithNodesTotal += rhs.intersectionTestsWithNodes;
	intersectionTestsWithTrianglesTotal += rhs.intersectionTestsWithTriangles;
	fallbackBvhSearchesTotal += rhs.fallbackBvhSearch;
	intersectionTestsNonFallbackTotal += rhs.intersectionTestsTotalNonFallback;
	intersectionTestsWithNodesNonFallbackTotal += rhs.intersectionTestsWithNodesNonFallback;
	intersectionTestsWithTrianglesNonFallbackTotal += rhs.intersectionTestsWithTrianglesNonFallback;
	TIME(timeTraversingTotal += rhs.traversalTime;);
	if (rhs.hit()) {
		intersectionTestsWhenHitTotal += rhs.intersectionTestsWhenHit;
		intersectionTestsWithNodesWhenHitTotal += rhs.intersectionTestsWithNodesWhenHit;
		intersectionTestsWithTrianglesWhenHitTotal += rhs.intersectionTestsWithTrianglesWhenHit;
		intersectionTestsWhenHitNonFallbackTotal += rhs.intersectionTestsWhenHitNonFallback;
		intersectionTestsWithNodesWhenHitNonFallbackTotal += rhs.intersectionTestsWithNodesWhenHitNonFallback;
		intersectionTestsWithTrianglesWhenHitNonFallbackTotal += rhs.intersectionTestsWithTrianglesWhenHitNonFallback;
	}

	return *this;
}

RayCasterResults& pah::RayCasterResults::operator+=(const Bvh::TraversalResults& rhs) {
	raysAmount++;
	bvhsTraversedTotal++;
	hitsTotal += rhs.hit();
	missesTotal += !rhs.hit();
	intersectionTestsTotal += rhs.intersectionTestsTotal;
	intersectionTestsWithNodesTotal += rhs.intersectionTestsWithNodes;
	intersectionTestsWithTrianglesTotal += rhs.intersectionTestsWithTriangles;
	intersectionTestsNonFallbackTotal += rhs.intersectionTestsTotal;
	intersectionTestsWithNodesNonFallbackTotal += rhs.intersectionTestsWithNodes;
	intersectionTestsWithTrianglesNonFallbackTotal += rhs.intersectionTestsWithTriangles;
	TIME(timeTraversingTotal += rhs.traversalTime;);
	if (rhs.hit()) {
		intersectionTestsWhenHitTotal += rhs.intersectionTestsTotal;
		intersectionTestsWithNodesWhenHitTotal += rhs.intersectionTestsWithNodes;
		intersectionTestsWithTrianglesWhenHitTotal += rhs.intersectionTestsWithTriangles;
		intersectionTestsWhenHitNonFallbackTotal += rhs.intersectionTestsTotal;
		intersectionTestsWithNodesWhenHitNonFallbackTotal += rhs.intersectionTestsWithNodes;
		intersectionTestsWithTrianglesWhenHitNonFallbackTotal += rhs.intersectionTestsWithTriangles;
	}
	return *this;
}

CumulativeRayCasterResults pah::operator+(const RayCasterResults& lhs, const RayCasterResults& rhs) {
	CumulativeRayCasterResults crcr{};
	crcr.raysAmount = lhs.raysAmount + rhs.raysAmount;
	crcr.rayCastersAmount = 2;
	crcr.bvhsTraversedTotal = lhs.bvhsTraversedTotal + rhs.bvhsTraversedTotal;
	crcr.hitsTotal = lhs.hitsTotal + rhs.hitsTotal;
	crcr.missesTotal = lhs.missesTotal + rhs.missesTotal;
	crcr.intersectionTestsTotal = lhs.intersectionTestsTotal + rhs.intersectionTestsTotal;
	crcr.intersectionTestsWithNodesTotal = lhs.intersectionTestsWithNodesTotal + rhs.intersectionTestsWithNodesTotal;
	crcr.intersectionTestsWithTrianglesTotal = lhs.intersectionTestsWithTrianglesTotal + rhs.intersectionTestsWithTrianglesTotal;
	crcr.intersectionTestsWhenHitTotal = lhs.intersectionTestsWhenHitTotal + rhs.intersectionTestsWhenHitTotal;
	crcr.intersectionTestsWithNodesWhenHitTotal = lhs.intersectionTestsWithNodesWhenHitTotal + rhs.intersectionTestsWithNodesWhenHitTotal;
	crcr.intersectionTestsWithTrianglesWhenHitTotal = lhs.intersectionTestsWithTrianglesWhenHitTotal + rhs.intersectionTestsWithTrianglesWhenHitTotal;
	TIME(crcr.timeTraversingTotal = lhs.timeTraversingTotal + rhs.timeTraversingTotal;);

	return crcr;
}

