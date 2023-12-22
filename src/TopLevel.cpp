#include "TopLevel.h"

#include <ranges>
#include <exception>

using namespace std;

const vector<pah::Bvh>& pah::TopLevel::getBvhs() const {
	return bvhs;
}

const vector<pah::Triangle>& pah::TopLevel::getTriangles() const {
	return triangles;
}

void pah::TopLevelAabbs::build() {
	vector<vector<const Triangle*>> bvhsTriangles(bvhs.size());
	
	//understand the BVHs each triangle is contained into
	for (auto& t : triangles) {
		for (int i = 0; i < bvhs.size(); ++i) {
			auto& region = bvhs[i].getInfluenceArea().getBvhRegion();
			if (region.contains(t.v1) || region.contains(t.v2) || region.contains(t.v3)) {
				bvhsTriangles[i].push_back(&t);
			}
		}
	}

	//build the BVHs with the corresponding triangles
	for (int i = 0; i < bvhs.size(); ++i) {
		bvhs[i].build(bvhsTriangles[i]);
	}
}

void pah::TopLevelAabbs::update() {
	throw exception{ "TopLevelAabbs::update function not implemented yet." };
}

vector<pah::Bvh*> pah::TopLevelAabbs::containedIn(const Vector3& point) {
	vector<Bvh*> containedIn;
	for (auto& bvh : bvhs) {
		if (bvh.getInfluenceArea().getBvhRegion().contains(point)) {
			containedIn.push_back(&bvh);
		}
	}
	return containedIn;
}


void pah::TopLevelOctree::build() {
	auto bvhsPointers = bvhs | std::views::transform([](Bvh& bvh) { return &bvh; }) | std::ranges::to<vector>(); //make vector of pointers
	buildRecursive(*root, bvhsPointers, {});
}

void pah::TopLevelOctree::update() {
	throw exception{ "TopLevelOctree::update function not implemented yet." };
}

std::vector<pah::Bvh*> pah::TopLevelOctree::containedIn(const Vector3& point) {
	Node* current = &*root;
	while (!current->isLeaf()) {
		auto center = current->aabb.center();
		int index = positionToIndex(point.x > center.x, point.y > center.y, point.z > center.z); //get the index based on the position of the point (point is assumed to be inside the current node AABB)
		current = &*current->children[index];
	}
	return current->bvhs;
}

pah::TopLevelOctree::Node& pah::TopLevelOctree::getRoot() const {
	return *root;
}

void pah::TopLevelOctree::buildRecursive(Node& node, const vector<Bvh*>& fatherCollidingRegions, const vector<Bvh*>& fatherFullyContainedRegions, int currentLevel) {
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
		for (int i = 0; i < 8; ++i) {
			auto& child = node.children[i];
			Vector3 half = node.aabb.center();
			Vector3 position = indexToPosition(i); //the bottommost, downmost, backwardmost octant is represented by <0,0,0>, the opposite by <1,1,1>, and everything in between
			Aabb childAabb{
				node.aabb.min + half * position,
				node.aabb.min - half * -position
			};
			child = make_unique<Node>(childAabb); //create empty child
			buildRecursive(*child, collidingRegions, node.bvhs, ++currentLevel); //build child
		}
	}
	//At the moment the node only contains the regions that fully contain it: our approach is conservative.
	//We prefer to have slightly smaller regions than slightly bigger ones, since it is likely that, at the border of the region, it wouldn't be useful to look in the local BVH first
}

int pah::TopLevelOctree::positionToIndex(bool x, bool y, bool z) {
	return 0 | (x >> 2) | (y >> 1) | z;
}

int pah::TopLevelOctree::positionToIndex(const Vector3& pos) {
	return positionToIndex(pos.x != 0, pos.y != 0, pos.z != 0);
}

pah::Vector3 pah::TopLevelOctree::indexToPosition(int i) {
	bool forward = (i >> 0) & 1, upward = (i >> 1) & 1, rightward = ((i >> 2) & 1);
	return Vector3{ rightward, upward, forward };
}
