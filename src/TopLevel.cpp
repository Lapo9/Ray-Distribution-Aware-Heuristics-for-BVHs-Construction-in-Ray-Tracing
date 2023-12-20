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
	
	//understand the BVHs each triangle is into
	for (auto& t : triangles) {
		for (int i = 0; i < bvhs.size(); ++i) {
			auto& region = bvhs[i].getInfluenceArea().getBvhRegion();
			if (region.isInside(t.v1) || region.isInside(t.v2) || region.isInside(t.v3)) {
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
		if (bvh.getInfluenceArea().getBvhRegion().isInside(point)) {
			containedIn.push_back(&bvh);
		}
	}
	return containedIn;
}
