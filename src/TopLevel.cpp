#include "TopLevel.h"

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

}

void pah::TopLevelOctree::update() {
	throw exception{ "TopLevelOctree::update function not implemented yet." };
}

std::vector<pah::Bvh*> pah::TopLevelOctree::containedIn(const Vector3& point) {
	Node* current = &*root;
	while (!current->isLeaf()) {
		int index = 0;
		current = &*current->children[index];
	}
	return current->bvhs;
}


void pah::TopLevelOctree::buildRecursive(Node& node, const vector<Bvh*>& fatherCollidingRegions, const vector<Bvh*>& fatherFullyContainedRegions) {
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
	node.setLeaf(leafNode);
	if (node.isLeaf()) {
		for (auto& child : node.children) {
			buildRecursive(*child, collidingRegions, node.bvhs);
		}
	}
	//At the moment the node only contains the regions that fully contain it: our approach is conservative.
	//We prefer to have slightly smaller regions than slightly bigger ones, since it is likely that, at the border of the region, it wouldn't be useful to look in the local BVH first
}
