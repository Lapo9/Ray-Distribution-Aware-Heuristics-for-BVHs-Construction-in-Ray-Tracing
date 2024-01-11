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
void pah::TopLevel::build() {
	unordered_map<const pah::Bvh*, vector<const Triangle*>> bvhsTriangles; //maps the BVH and the triangles it contains

	//understand the BVHs each triangle is contained into
	for (auto& t : triangles) {
		unordered_set<const pah::Bvh*> containedInto; //the BVHs where thre triangle is contained into

		//for each vertex, get the BVHs it is contained into, and add them to the set (we directly use the containedIn function, so this is polymorphic
		containedInto.insert_range(containedIn(t.v0));
		containedInto.insert_range(containedIn(t.v1));
		containedInto.insert_range(containedIn(t.v2));

		//add the triangle to each BVH where it is contained into (at least one vertex)
		for (const auto& bvh : containedInto) bvhsTriangles[bvh].push_back(&t);
	}

	//build the BVHs with the corresponding triangles
	for (auto& bvh : bvhs) {
		bvh.build(bvhsTriangles[&bvh]);
	}
}

TopLevel::TraversalInfo pah::TopLevel::traverse(const TopLevel& topLevel, const Ray& ray) {
	TraversalInfo res{};
	TIME(TimeLogger timeLogger{ [&res](auto duration) {res.traversalTime = duration; } });

	const auto& relevantBvhs = topLevel.containedIn(ray.getOrigin()); //here we have the BVHs where the starting point of the ray is contained (we don't know if the direction is relevant tho)
	res.totalBvhs = relevantBvhs.size();

	for (const auto& bvh : relevantBvhs) {
		//here we check that the direction of the ray is relevant to this particuar BVH
		if (bvh->getInfluenceArea().isDirectionAffine(ray.getDirection(), TOLERANCE)) { //TODO tolerance here should be a bigger value (we have to tune it)
			res.bvhsTraversed++;
			const auto& results = bvh->traverse(ray);
			if (results.hit()) {
				res.closestHit = results.closestHit;
				res.closestHitDistance = results.closestHitDistance;
				break;
			}
		}
	}

	timeLogger.stop();
	return res;
}

const vector<pah::Bvh>& pah::TopLevel::getBvhs() const {
	return bvhs;
}

const vector<pah::Triangle>& pah::TopLevel::getTriangles() const {
	return triangles;
}

void pah::TopLevelAabbs::build() {
	TopLevel::build();
}

void pah::TopLevelAabbs::update() {
	throw exception{ "TopLevelAabbs::update function not implemented yet." };
}

vector<const pah::Bvh*> pah::TopLevelAabbs::containedIn(const Vector3& point) const {
	vector<const Bvh*> containedIn;
	for (auto& bvh : bvhs) {
		if (bvh.getInfluenceArea().getBvhRegion().contains(point)) {
			containedIn.push_back(&bvh);
		}
	}
	return containedIn;
}


// ======| TopLevelOctree |======
void pah::TopLevelOctree::build() {
	INFO(TimeLogger timeLoggerTotalBuild{ [this](DurationMs duration) { totalBuildTime = duration; } });

	//build the octree
	auto bvhsPointers = bvhs | std::views::transform([](Bvh& bvh) { return &bvh; }) | std::ranges::to<vector>(); //make vector of pointers
	buildOctreeRecursive(*root, bvhsPointers, {});

	//build the BVHs
	TopLevel::build();
}

void pah::TopLevelOctree::update() {
	throw exception{ "TopLevelOctree::update function not implemented yet." };
}

vector<const pah::Bvh*> pah::TopLevelOctree::containedIn(const Vector3& point) const {
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
		if (bvh->getInfluenceArea().getBvhRegion().isCollidingWith(node.aabb)) {
			if (bvh->getInfluenceArea().getBvhRegion().fullyContains(node.aabb)) {
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

