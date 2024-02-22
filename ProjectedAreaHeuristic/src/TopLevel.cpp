#include "TopLevel.h"

#include <ranges>
#include <unordered_set>
#include <unordered_map>
#include <exception>

#include "Utilities.h"

using namespace std;
using namespace pah;
using namespace pah::utilities;


// ======| TopLevel |======
void pah::TopLevel::build(float splitPlaneQualityThreshold, float maxChildrenFatherHitProbabilityRatio) {
	unordered_map<const pah::Bvh*, vector<const Triangle*>> bvhsTriangles; //maps the BVH and the triangles it contains
	//build the fallback BVH with all the triangles
	fallbackBvh.build(triangles | views::transform([](const Triangle& t) {return &t; }) | ranges::to<vector>(), splitPlaneQualityThreshold, maxChildrenFatherHitProbabilityRatio);

	//understand the BVHs each triangle is contained into
	for (const auto& t : triangles) {
		unordered_set<const pah::Bvh*> containedInto; //the BVHs the triangle is contained into

		//for each vertex, get the BVHs it is contained into, and add them to the set (we directly use the containedIn function, so this is polymorphic
		containedInto.insert_range(containedIn(t.v0));
		containedInto.insert_range(containedIn(t.v1));
		containedInto.insert_range(containedIn(t.v2));

		//add the triangle to each BVH where it is contained into (at least one vertex)
		for (const auto& bvh : containedInto) bvhsTriangles[bvh].push_back(&t);
	}

	//build the BVHs with the corresponding triangles
	for (auto& bvh : bvhs) {
		bvh.build(bvhsTriangles[&bvh], splitPlaneQualityThreshold, maxChildrenFatherHitProbabilityRatio);
	}
}

TopLevel::TraversalResults pah::TopLevel::traverse(const Ray& ray) const {
	TraversalResults res{};
	TIME(TimeLogger timeLogger{ [&res](auto duration) {res.traversalTime = duration; } });

	const auto& relevantBvhs = containedIn(ray.getOrigin()); //here we have the BVHs where the starting point of the ray is contained (we don't know if the direction is relevant tho)
	res.totalBvhs = relevantBvhs.size();

	for (const auto& bvh : relevantBvhs) {
		//here we check that the direction of the ray is relevant to this particuar BVH
		if (bvh->getInfluenceArea()->isDirectionAffine(ray, TOLERANCE)) { //TODO tolerance here should be a bigger value (we have to tune it)
			const auto& results = bvh->traverse(ray);
			res += results; //here we sum some attributes such as the number of intersections and the traversal cost
			if (results.hit()) {
				res.closestHit = results.closestHit;
				res.closestHitDistance = results.closestHitDistance;
				break; //if we find an hit, we don't need to traverse the remaining BVHs
			}
		}
	}

	//if we couldn't find a hit, we fallback to the global BVH
	if (!res.hit()) {
		res.fallbackBvhSearch = true;
		const auto& results = fallbackBvh.traverse(ray);
		res += results; //here we sum some attributes such as the number of intersections and the traversal cost
		if (results.hit()) {
			res.closestHit = results.closestHit;
			res.closestHitDistance = results.closestHitDistance;
		}
	}

	INFO(timeLogger.stop(););
	return res;
}

const vector<pah::Bvh>& pah::TopLevel::getBvhs() const {
	return bvhs;
}

const vector<pah::Triangle>& pah::TopLevel::getTriangles() const {
	return triangles;
}


// ======| TopLevelAabbs |======
void pah::TopLevelAabbs::build(float splitPlaneQualityThreshold, float maxChildrenFatherHitProbabilityRatio) {
	TopLevel::build(splitPlaneQualityThreshold, maxChildrenFatherHitProbabilityRatio);
}

void pah::TopLevelAabbs::update() {
	throw exception{ "TopLevelAabbs::update function not implemented yet." };
}

vector<const pah::Bvh*> pah::TopLevelAabbs::containedIn(const Vector3& point) const {
	vector<const Bvh*> containedIn;
	for (auto& bvh : bvhs) {
		if (bvh.getInfluenceArea()->getBvhRegion().contains(point)) {
			containedIn.push_back(&bvh);
		}
	}
	return containedIn;
}


// ======| TopLevelOctree |======
void pah::TopLevelOctree::build(float splitPlaneQualityThreshold, float maxChildrenFatherHitProbabilityRatio) {
	INFO(TimeLogger timeLoggerTotalBuild{ [this](DurationMs duration) { totalBuildTime = duration; } });

	//build the octree
	auto bvhsPointers = bvhs | std::views::transform([](Bvh& bvh) { return &bvh; }) | std::ranges::to<vector>(); //make vector of pointers
	buildOctreeRecursive(*root, bvhsPointers, {});

	//build the BVHs
	TopLevel::build(splitPlaneQualityThreshold, maxChildrenFatherHitProbabilityRatio);
}

void pah::TopLevelOctree::update() {
	throw exception{ "TopLevelOctree::update function not implemented yet." };
}

vector<const pah::Bvh*> pah::TopLevelOctree::containedIn(const Vector3& point) const {
	//if the point is outside the region covered by the octree, it is useless to continue the search
	if (!root->aabb.contains(point)) return {};

	Node* current = &*root;
	while (!current->isLeaf()) {
		auto center = current->aabb.center();
		int index = positionToIndex(point.x > center.x, point.y > center.y, point.z > center.z); //get the index based on the position of the point (point is assumed to be inside the current node AABB)
		current = &*current->children[index];
	}
	return vector<const Bvh*>{ current->bvhs.begin(), current->bvhs.end() };
}

TopLevelOctree::Node& pah::TopLevelOctree::getRoot() const {
	return *root;
}

INFO(const DurationMs pah::TopLevelOctree::getTotalBuildTime() const {
	return totalBuildTime;
})

TopLevelOctree::Properties pah::TopLevelOctree::getProperties() const {
	return properties;
}

void pah::TopLevelOctree::buildOctreeRecursive(Node& node, const vector<Bvh*>& fatherCollidingRegions, const vector<Bvh*>& fatherFullyContainedRegions, int currentLevel) {
	TIME(TimeLogger timeLoggerTotal{ [&timingInfo = node.timingInfo](DurationMs duration) { timingInfo.logTotal(duration); } };);

	vector<Bvh*> collidingRegions;
	node.bvhs = fatherFullyContainedRegions; //if the father node was fully contained in some region, for sure the child will also be
	bool leafNode = true; //this will become false if there is a region that intersects the node, but doesn't fully contain it

	//for each region that collided with the father, but didn't fully contained it, we have to check what happens with the child
	for (const auto& bvh : fatherCollidingRegions) {
		if (bvh->getInfluenceArea()->getBvhRegion().isCollidingWith(node.aabb)) {
			if (bvh->getInfluenceArea()->getBvhRegion().fullyContains(node.aabb)) {
				node.bvhs.push_back(bvh);
			}
			else {
				collidingRegions.push_back(bvh);
				leafNode = false;
			}
		}
	}

	//the node is a leaf if there are no colliding but not fully contained regions, or if we reached the max level
	node.setLeaf(leafNode || currentLevel >= properties.maxLevel);
	if (!node.isLeaf()) {
		currentLevel++;
		for (int i = 0; i < 8; ++i) {
			auto& child = node.children[i];
			Vector3 halfExtents = (node.aabb.max - node.aabb.min) / 2.0f;
			Vector3 position = indexToPosition(i); //the bottommost, downmost, backwardmost octant is represented by <0,0,0>, the opposite by <1,1,1>, and everything in between
			Aabb childAabb{
				node.aabb.min + halfExtents * position,
				node.aabb.max - halfExtents * (Vector3{1.0f, 1.0f, 1.0f} - position)
			};
			child = make_unique<Node>(childAabb); //create empty child
			TIME(timeLoggerTotal.pause();); //in the time for this node, we don't want to include the time used to build all its descendents
			buildOctreeRecursive(*child, collidingRegions, node.bvhs, currentLevel); //build child
			TIME(timeLoggerTotal.resume(););
		}
	}

	//If conservativeApproach is true, the node only contains the regions that fully contain it.
	//In this way we have slightly smaller regions than the original, since it is likely that, at the border of the region, it wouldn't be useful to look in the local BVH first.
	//On the other hand, this can give rise to not fully connected regions.
	if(!properties.conservativeApproach) node.bvhs.append_range(collidingRegions);
}

const Bvh& pah::TopLevel::getFallbackBvh() const {
	return fallbackBvh;
}

int pah::TopLevelOctree::positionToIndex(bool x, bool y, bool z) {
	return 0 | (x << 2) | (y << 1) | z;
}

int pah::TopLevelOctree::positionToIndex(const Vector3& pos) {
	return positionToIndex(pos.x != 0, pos.y != 0, pos.z != 0);
}

Vector3 pah::TopLevelOctree::indexToPosition(int i) {
	bool forward = (i >> 0) & 1, upward = (i >> 1) & 1, rightward = ((i >> 2) & 1);
	return Vector3{ rightward, upward, forward };
}

