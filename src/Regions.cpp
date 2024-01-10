#include "Regions.h"

#include "Projections.h"

using namespace std;
using namespace glm;
using namespace pah;


	// ======| Aabb |======
pah::Aabb::Aabb(const vector<const Triangle*>&triangles) : min{ numeric_limits<float>::max() }, max{ -numeric_limits<float>::max() } {
	for (const auto& t : triangles) {
		if (t->v1.x < min.x) min.x = t->v1.x;
		if (t->v2.x < min.x) min.x = t->v2.x;
		if (t->v3.x < min.x) min.x = t->v3.x;

		if (t->v1.y < min.y) min.y = t->v1.y;
		if (t->v2.y < min.y) min.y = t->v2.y;
		if (t->v3.y < min.y) min.y = t->v3.y;

		if (t->v1.z < min.z) min.z = t->v1.z;
		if (t->v2.z < min.z) min.z = t->v2.z;
		if (t->v3.z < min.z) min.z = t->v3.z;


		if (t->v1.x > max.x) max.x = t->v1.x;
		if (t->v2.x > max.x) max.x = t->v2.x;
		if (t->v3.x > max.x) max.x = t->v3.x;

		if (t->v1.y > max.y) max.y = t->v1.y;
		if (t->v2.y > max.y) max.y = t->v2.y;
		if (t->v3.y > max.y) max.y = t->v3.y;

		if (t->v1.z > max.z) max.z = t->v1.z;
		if (t->v2.z > max.z) max.z = t->v2.z;
		if (t->v3.z > max.z) max.z = t->v3.z;
	}
}

pah::Aabb::Aabb(const Vector3 & min, const Vector3 & max) : min{ min }, max{ max } {}

bool pah::Aabb::contains(const Vector3 & point) const {
	return
		point.x >= min.x &&
		point.y >= min.y &&
		point.z >= min.z &&
		point.x <= max.x &&
		point.y <= max.y &&
		point.z <= max.z;
}

Aabb pah::Aabb::enclosingAabb() const {
	return *this;
}

bool pah::Aabb::isCollidingWith(const Aabb & aabb) const {
	return collisionDetection::areColliding(*this, aabb);
}

bool pah::Aabb::fullyContains(const Aabb & aabb) const {
	return
		min.x <= aabb.min.x &&
		min.y <= aabb.min.y &&
		min.z <= aabb.min.z &&
		max.x >= aabb.max.x &&
		max.y >= aabb.max.y &&
		max.z >= aabb.max.z;
}

Vector3 pah::Aabb::center() const {
	return (min + max) / 2.0f;
}

Vector3 pah::Aabb::size() const {
	return (max - min);
}

float pah::Aabb::surfaceArea() const {
	return 2.0f * (
		(max.x - min.x) * (max.y - min.y) +
		(max.x - min.x) * (max.z - min.z) +
		(max.y - min.y) * (max.z - min.z));
}

array<Vector3, 8> pah::Aabb::getPoints() const {
	array<Vector3, 8> points{};
	for (int i = 0; i < 2; ++i)
		for (int j = 0; j < 2; ++j)
			for (int k = 0; k < 2; ++k) {
				Vector3 vertex{ i, j, k }; //selects which vertex we are considering (e.g. top-left-back)
				points[i * 4 + j * 2 + k] = min + vertex * size();
			}
	return points;
}

Vector3 pah::Aabb::getPoint(int i) const {
	int frontward = (i >> 0) & 1;
	int upward = (i >> 1) & 1;
	int rightward = (i >> 2) & 1;
	return min + (max - min) * Vector3 { rightward, upward, frontward };
}

pah::Aabb pah::Aabb::maxAabb() {
	Vector3 min{ -numeric_limits<float>::max() };
	Vector3 max{ numeric_limits<float>::max() };
	return Aabb{ min, max };
}

pah::Aabb pah::Aabb::minAabb() {
	Vector3 min{ numeric_limits<float>::max() };
	Vector3 max{ -numeric_limits<float>::max() };
	return Aabb{ min, max };
}

Aabb& pah::Aabb::operator+=(const Aabb & lhs) {
	min = glm::min(min, lhs.min);
	max = glm::max(max, lhs.max);
	return *this;
}

pah::Aabb& pah::operator+(Aabb lhs, const Aabb & rhs) {
	return lhs += rhs;
}


// ======| Obb |======
pah::Obb::Obb(const Vector3 & center, const Vector3 & halfSize, const Vector3 & forward) : center{ center }, halfSize{ halfSize },
	forward{ normalize(forward) }, right{ cross(Vector3{0,1,0}, forward) }, up{ cross(forward, right) } {
}

bool pah::Obb::contains(const Vector3 & point) const {
	using namespace glm;

	Vector3 d = point - center;
	//we project the distance vector between the point and the center in the reference system of the OBB, and we check the distances are < than the half size in all directions
	return
		abs(dot(d, right)) <= halfSize.x &&
		abs(dot(d, up)) <= halfSize.y &&
		abs(dot(d, forward)) <= halfSize.z;
}

Aabb pah::Obb::enclosingAabb() const {
	Aabb aabb = Aabb::minAabb();
	auto vertices = getPoints();

	//TODO probably there is a more efficient way
	for (auto& v : vertices) {
		//update max and min of enclosing AABB
		aabb.max = max(aabb.max, v);
		aabb.min = min(aabb.min, v);
	}
	return aabb;
}

bool pah::Obb::isCollidingWith(const Aabb & aabb) const {
	return collisionDetection::areColliding(*this, aabb);
}

bool pah::Obb::fullyContains(const Aabb & aabb) const {
	const auto& vertices = aabb.getPoints();
	for (const auto& vertex : vertices) {
		if (!contains(vertex)) return false;
	}
	return true;
}

array<Vector3, 8> pah::Obb::getPoints() const {
	array<Vector3, 8> points{};
	//loop through each vertex of the OBB
	for (int i = -1; i <= 1; i += 2)
		for (int j = -1; j <= 1; j += 2)
			for (int k = -1; k <= 1; k += 2) {
				Vector3 obbVertex = halfSize * Vector3{ i,j,k }; //point in the reference system of the OBB
				Matrix3 rotation{ right, up, forward };
				Vector3 worldVertex = rotation * obbVertex + center; //point in world space
				points[((i + 1) / 2) * 4 + ((j + 1) / 2) * 2 + ((k + 1) / 2) * 1] = worldVertex;
			}
	return points;
}


	// ======| AabbForObb |======
pah::AabbForObb::AabbForObb(const Vector3 & center, const Vector3 & halfSize, const Vector3 & forward) : obb{ center, halfSize, forward }, aabb{ obb.enclosingAabb() } {}

pah::AabbForObb::AabbForObb(const Obb & obb) : obb{ obb }, aabb{ obb.enclosingAabb() } {}

bool pah::AabbForObb::contains(const Vector3 & point) const {
	//check if the point is inside the AABB (cheap), if it is, check the OBB
	if (aabb.contains(point)) return obb.contains(point);
	return false;
}

Aabb pah::AabbForObb::enclosingAabb() const {
	return aabb;
}

bool pah::AabbForObb::isCollidingWith(const Aabb & aabb) const {
	return collisionDetection::areColliding(*this, aabb);
}

bool pah::AabbForObb::fullyContains(const Aabb & aabb) const {
	if (!this->aabb.fullyContains(aabb)) return false;
	return this->obb.fullyContains(aabb);
}


// ======| Frustum |======
pah::Frustum::Frustum(const Matrix4 & viewProjectionMatrix) : viewProjectionMatrix{ viewProjectionMatrix } {
	fillVertices();
	fillEdgesDirection();
	fillFacesNormals();
	fillEnclosingAabb();
}

pah::Frustum::Frustum(const Pov & pov, float far, float near, float fovX, float fovY)
	: Frustum{ projection::computePerspectiveMatrix(far, near, { fovX, fovY }) * projection::computeViewMatrix(pov) } {}

bool pah::Frustum::contains(const Vector3 & point) const {
	if (!enclosingAabbObj.contains(point)) return false;

	//look at "Fast Extraction of Viewing Frustum Planes from the WorldView-Projection Matrix" section 2 and the GeoGebra file "2dFrustum"
		//basically p' = M*p --> p inside frustum iff -w' < x' < w' AND -w' < y' < w' AND -w' < z' < w'
	auto p = viewProjectionMatrix * Vector4{ point, 1.0f }; //projected point
	return -p.w < p.x && p.x < p.w &&
		-p.w < p.y && p.y < p.w &&
		-p.w < p.z && p.z < p.w;
}

pah::Aabb pah::Frustum::enclosingAabb() const {
	return enclosingAabbObj;
}

bool pah::Frustum::isCollidingWith(const Aabb & aabb) const {
	return collisionDetection::areColliding(*this, aabb);
}

bool pah::Frustum::fullyContains(const Aabb & aabb) const {
	if (!enclosingAabbObj.fullyContains(aabb)) return false;

	for (const auto& vertex : vertices) {
		if (!contains(vertex)) return false;
	}
	return true;
}

array<Vector3, 8> pah::Frustum::getPoints() const {
	return vertices;
}

array<Vector3, 6> pah::Frustum::getEdgesDirections() const {
	return edgesDirections;
}

array<Vector3, 6> pah::Frustum::getFacesNormals() const {
	return facesNormals;
}

pah::Matrix4 pah::Frustum::getViewProjectionMatrix() const {
	return viewProjectionMatrix;
}

const Vector3& pah::Frustum::getRight() const {
	return edgesDirections[0];
}

const Vector3& pah::Frustum::getUp() const {
	return edgesDirections[1];
}

const Vector3& pah::Frustum::getForward() const {
	return facesNormals[5];
}

Matrix4 pah::Frustum::extractViewMatrix() const {
	//the center in view space is always (0,0,-1,0)
	Vector4 centerView{ 0,0,-1,0 };
	//we know that P' = M * P ==> P = M^(-1) * P' where P is a point in world space, P' in view space and M is the view projection matrix
	Vector4 centerWorld = glm::inverse(viewProjectionMatrix) * centerView;
	centerWorld /= centerWorld.w; //we want 1 as the w component in homogeneous coordinates
	//we know that M = P * V ==> P = M * V^(-1)
	return projection::computeViewMatrix(Pov{ centerWorld, getForward() });
}

Matrix4 pah::Frustum::extractProjectionMatrix() const {
	Matrix4 viewMatrix = extractViewMatrix();
	Matrix4 inverseViewMatrix = glm::inverse(viewMatrix);
	return viewProjectionMatrix * inverseViewMatrix;
}

projection::ProjectionMatrixParameters pah::Frustum::getViewProjectionMatrixParameters() const {
	return projection::extractPerspectiveMatrixParameters(extractProjectionMatrix());
}

void pah::Frustum::fillFacesNormals() {
	//look at "Fast Extraction of Viewing Frustum Planes from the WorldView-Projection Matrix" section 2
	using namespace glm;
	const auto& M = viewProjectionMatrix;

	//left
	facesNormals[0] = -normalize(Vector3{
		M[0][3] + M[0][0],
		M[1][3] + M[1][0],
		M[2][3] + M[2][0]
								 });

	//right
	facesNormals[1] = -normalize(Vector3{
		M[0][3] - M[0][0],
		M[1][3] - M[1][0],
		M[2][3] - M[2][0]
								 });

	//bottom
	facesNormals[2] = -normalize(Vector3{
		M[0][3] + M[0][1],
		M[1][3] + M[1][1],
		M[2][3] + M[2][1]
								 });

	//top
	facesNormals[3] = -normalize(Vector3{
		M[0][3] - M[0][1],
		M[1][3] - M[1][1],
		M[2][3] - M[2][1]
								 });

	//near
	facesNormals[4] = -normalize(Vector3{
		M[0][3] + M[0][2],
		M[1][3] + M[1][2],
		M[2][3] + M[2][2]
								 });

	//far
	facesNormals[5] = -normalize(Vector3{
		M[0][3] - M[0][2],
		M[1][3] - M[1][2],
		M[2][3] - M[2][2]
								 });
}

void pah::Frustum::fillEdgesDirection() {
	edgesDirections[0] = normalize(vertices[4] - vertices[0]); //right
	edgesDirections[1] = normalize(vertices[2] - vertices[0]); //top

	//oblique ones
	edgesDirections[2] = normalize(vertices[1] - vertices[0]);
	edgesDirections[3] = normalize(vertices[3] - vertices[2]);
	edgesDirections[4] = normalize(vertices[5] - vertices[4]);
	edgesDirections[5] = normalize(vertices[7] - vertices[6]);

}

void pah::Frustum::fillVertices() {
	//we know that the vertices of the frustum in clipping space (after the application of the view-projection matrix) are the vertices of the unit cube.
		//therefore we "un-apply" the view-projection matrix (we apply its inverse) and then we use the vertices with w = 1 (we divide by w).
	const auto& Mi = inverse(viewProjectionMatrix);

	for (int i = -1, count = 0; i <= 1; i += 2)
		for (int j = -1; j <= 1; j += 2)
			for (int k = -1; k <= 1; k += 2) {
				Vector4 clippingSpaceVertex{ k,j,i,1 };
				Vector4 homogeneousVertex = Mi * clippingSpaceVertex;
				vertices[count] = Vector3{ homogeneousVertex } / homogeneousVertex.w;
				count++;
			}
}

void pah::Frustum::fillEnclosingAabb() {
	enclosingAabbObj = Aabb::minAabb();

	for (const auto& vertex : vertices) {
		//update max and min of enclosing AABB
		enclosingAabbObj.max = max(enclosingAabbObj.max, vertex);
		enclosingAabbObj.min = min(enclosingAabbObj.min, vertex);
	}
}


// ======| namesapce collisionDetection |======
static bool collisionDetection::areColliding(const Aabb & aabb1, const Aabb & aabb2) {
	return
		aabb1.min.x <= aabb2.max.x &&
		aabb1.max.x >= aabb2.min.x &&
		aabb1.min.y <= aabb2.max.y &&
		aabb1.max.y >= aabb2.min.y &&
		aabb1.min.z <= aabb2.max.z &&
		aabb1.max.z >= aabb2.min.z;
}

static bool collisionDetection::areColliding(const AabbForObb & aabbForObb, const Aabb & aabb) {
	//first, we check if the enclosing AABB of the OBB overlaps with the AABB (it can save a lot of time)
	bool aabbsColliding = areColliding(aabb, aabbForObb.enclosingAabb());
	if (!aabbsColliding) return false;

	//then we check whether the OBB is "almost" an AABB (in this case we can approximate the collision to the AABB v AABB case)
	const Obb& obb = aabbForObb.obb;
	if (almostAabb(obb)) return aabbsColliding;

	//else, we have to use SAT
	auto obbVertices = obb.getPoints();
	auto aabbVertices = aabb.getPoints();

	//these are the potential separating axes; we use lambdas in order to evaluate them lazily (important to avoid useless cross products in case of early outs)
	auto axes = vector<function<Vector3()>>{
		[]() { return Vector3{1,0,0}; },
		[]() { return Vector3{0,1,0}; },
		[]() { return Vector3{0,0,1}; },
		[&obb]() { return obb.right; },
		[&obb]() { return obb.up; },
		[&obb]() { return obb.forward; },
		[&obb]() { return cross(Vector3{1,0,0}, obb.right); },
		[&obb]() { return cross(Vector3{1,0,0}, obb.up); },
		[&obb]() { return cross(Vector3{1,0,0}, obb.forward); },
		[&obb]() { return cross(Vector3{0,1,0}, obb.right); },
		[&obb]() { return cross(Vector3{0,1,0}, obb.up); },
		[&obb]() { return cross(Vector3{0,1,0}, obb.forward); },
		[&obb]() { return cross(Vector3{0,0,1}, obb.right); },
		[&obb]() { return cross(Vector3{0,0,1}, obb.up); },
		[&obb]() { return cross(Vector3{0,0,1}, obb.forward); }
	};

	for (auto& getAxis : axes) {
		auto axis = getAxis();
		//get the points that, after the projection, will be the outermost (both for the OBB and the AABB)
		auto obbExtremesIndexes = projectedObbExtremes(axis, obb);
		auto aabbExtremesIndexes = projectedAabbExtremes(axis);

		// |--------------------|MA     MB    overlap iff mB <= MA && MB >= mA (not overlap iff mB > MA || MB < mA)
			// mA             mB|-----------|
		if (dot(aabbVertices[aabbExtremesIndexes.first], axis) > dot(obbVertices[obbExtremesIndexes.second], axis) ||
			dot(aabbVertices[aabbExtremesIndexes.second], axis) < dot(obbVertices[obbExtremesIndexes.first], axis)) return false;
	}
	return true; //if we havent't found any axis where there is no overlap, boxes are colliding
}

static bool collisionDetection::areColliding(const Obb & obb, const Aabb & aabb) {
	return areColliding(AabbForObb{ obb }, aabb);
}

static bool collisionDetection::areColliding(const Frustum & frustum, const Aabb & aabb) {
	using namespace std;
		using namespace glm;

	//first, we check if the enclosing AABB of the frustum overlaps with the AABB (it can save a lot of time)
	bool aabbsColliding = areColliding(aabb, frustum.enclosingAabb());
	if (!aabbsColliding) return false;

	//else, we have to use SAT
	auto frustumVertices = frustum.getPoints();
	auto aabbVertices = aabb.getPoints();

	//these are the potential separating axes; we use lambdas in order to evaluate them lazily (important to avoid useless cross products in case of early outs)
	auto axes = vector<function<Vector3()>>{
		[]() { return Vector3{1,0,0}; },
		[]() { return Vector3{0,1,0}; },
		[]() { return Vector3{0,0,1}; },
		[&frustum]() { return frustum.getFacesNormals()[0]; },
		[&frustum]() { return frustum.getFacesNormals()[1]; },
		[&frustum]() { return frustum.getFacesNormals()[2]; },
		[&frustum]() { return frustum.getFacesNormals()[3]; },
		[&frustum]() { return frustum.getFacesNormals()[4]; },
		[&frustum]() { return frustum.getFacesNormals()[5]; },
		[&frustum]() { return cross(Vector3{1,0,0}, frustum.getEdgesDirections()[0]); },
		[&frustum]() { return cross(Vector3{1,0,0}, frustum.getEdgesDirections()[1]); },
		[&frustum]() { return cross(Vector3{1,0,0}, frustum.getEdgesDirections()[2]); },
		[&frustum]() { return cross(Vector3{1,0,0}, frustum.getEdgesDirections()[3]); },
		[&frustum]() { return cross(Vector3{1,0,0}, frustum.getEdgesDirections()[4]); },
		[&frustum]() { return cross(Vector3{1,0,0}, frustum.getEdgesDirections()[5]); },
		[&frustum]() { return cross(Vector3{0,1,0}, frustum.getEdgesDirections()[0]); },
		[&frustum]() { return cross(Vector3{0,1,0}, frustum.getEdgesDirections()[1]); },
		[&frustum]() { return cross(Vector3{0,1,0}, frustum.getEdgesDirections()[2]); },
		[&frustum]() { return cross(Vector3{0,1,0}, frustum.getEdgesDirections()[3]); },
		[&frustum]() { return cross(Vector3{0,1,0}, frustum.getEdgesDirections()[4]); },
		[&frustum]() { return cross(Vector3{0,1,0}, frustum.getEdgesDirections()[5]); },
		[&frustum]() { return cross(Vector3{0,0,1}, frustum.getEdgesDirections()[0]); },
		[&frustum]() { return cross(Vector3{0,0,1}, frustum.getEdgesDirections()[1]); },
		[&frustum]() { return cross(Vector3{0,0,1}, frustum.getEdgesDirections()[2]); },
		[&frustum]() { return cross(Vector3{0,0,1}, frustum.getEdgesDirections()[3]); },
		[&frustum]() { return cross(Vector3{0,0,1}, frustum.getEdgesDirections()[4]); },
		[&frustum]() { return cross(Vector3{0,0,1}, frustum.getEdgesDirections()[5]); }
	};

	for (auto& getAxis : axes) {
		auto axis = getAxis();
		//get the points that, after the projection, will be the outermost (for the AABB we get the indexes, for the frustum the points themselves)
		auto aabbExtremesIndexes = projectedAabbExtremes(axis);
		auto frustumExtremes = projectedFrustumExtremes(axis, frustum);

		// |--------------------|MA     MB    overlap iff mB <= MA && MB >= mA (not overlap iff mB > MA || MB < mA)
			// mA             mB|-----------|
		if (dot(aabbVertices[aabbExtremesIndexes.first], axis) > frustumExtremes.second ||
			dot(aabbVertices[aabbExtremesIndexes.second], axis) < frustumExtremes.first) return false;
	}
	return true; //if we havent't found any axis where there is no overlap, boxes are colliding
}

pair<bool, float> pah::collisionDetection::areColliding(const Ray& ray, const Aabb& aabb) {
	logic_error("Function not implemented yet!"); //TODO implement this
}

pair<bool, float> pah::collisionDetection::areColliding(const Ray& ray, const Triangle& triangle) {
	
	const auto& N = triangle.normal(); //normal to the triangle
	const auto& R = ray.getDirection(); //direction of the ray
	Vector3 O{ 0.0f, 0.0f, 0.0f }; //origin

	//if the ray and the plane of the triangle are parallel, there is no intersection
	if (abs(dot(N, R)) < TOLERANCE) return { false, 0.0f };
	
	// P = O + tR is the parametric equation of the ray.
	// A * x + B * y + C * z + D = 0 is the equation of the plane where the triangle lies.
	// We know that any vertex of the triangle (e.g. V0) is on the plane, and that the normal to the plane N = (A, B, C), and D is the distance from the origin (O) to the plane.
	// Therefore D = -(Ax + By + Cz) = -(Nx * V0x + Ny * V0y + Nz * V0z) = -dot(N, V0)
	float D = -dot(N, triangle.v1);

	// To find the ray-plane intersection we can force the point of the plane to be P:
	// A * Px + B * Py + C * Pz + D = 0 --> A * (Ox + t*Rx) + B * (Oy + t*Ry) + C * (Oz + t*Rz) + D = 0 --> t = -(dot(N, O) + D) / dot(N, R)
	float t = -(dot(N, O) + D) / dot(N, R);

	// If the triangle is behind the ray origin, there is no intersection
	if (t < 0) return { false, 0.0f };

	// Now we have to find out whether the point is inside the triangle .
	// The point is inside iff it is "to the left" of all sides (taken in counter clockwise order).
	// If it is "to the left" of an edge, then the normal (N) and the cross product between the edge and the vector connecting the start of the edge and the point (P) should face the same direction (i.e. their dot product > 0).
	const auto& P = ray.getPosition() + R * t; //coordinates of the intersection point between the ray and the plane
	Vector3 e1 = triangle.v2 - triangle.v1; //vector representing a triangle edge
	Vector3 p1 = P - triangle.v1; //vector from the vertex of the triangle to the intersection point
	if (dot(N, cross(e1, p1)) <= 0) return { false, 0.0f };

	Vector3 e2 = triangle.v3 - triangle.v2;
	Vector3 p2 = P - triangle.v2;
	if (dot(N, cross(e2, p2)) <= 0) return { false, 0.0f };

	Vector3 e3 = triangle.v1 - triangle.v3;
	Vector3 p3 = P - triangle.v3;
	if (dot(N, cross(e3, p3)) <= 0) return { false, 0.0f };

	return { true, t };

	//TODO yet to be tested
}

static bool collisionDetection::almostParallel(const Vector3 & lhs, const Vector3 & rhs, float threshold) {
	return abs(dot(lhs, rhs)) > abs(length(lhs) * length(rhs)) - threshold * abs(length(lhs) * length(rhs)); //if vectors are parallel then a . b = ||a|| * ||b||
}

static bool collisionDetection::almostAabb(const Obb& obb) {
	return
		(almostParallel(obb.forward, Vector3{ 0,0,1 }) && almostParallel(obb.right, Vector3{ 1,0,0 })) ||
		(almostParallel(obb.forward, Vector3{ 0,0,1 }) && almostParallel(obb.up, Vector3{ 1,0,0 })) ||
		(almostParallel(obb.right, Vector3{ 0,0,1 }) && almostParallel(obb.forward, Vector3{ 1,0,0 })) ||
		(almostParallel(obb.right, Vector3{ 0,0,1 }) && almostParallel(obb.up, Vector3{ 1,0,0 })) ||
		(almostParallel(obb.up, Vector3{ 0,0,1 }) && almostParallel(obb.forward, Vector3{ 1,0,0 })) ||
		(almostParallel(obb.up, Vector3{ 0,0,1 }) && almostParallel(obb.right, Vector3{ 1,0,0 }));
}

static pair<int, int> collisionDetection::projectedAabbExtremes(const Vector3 & axis) {
	auto normalized = normalize(axis);
	float x = normalized.x, y = normalized.y, z = normalized.z;

	if (x >= 0 && y >= 0 && z <= 0) return { 1, 6 }; //yellow
	if (x <= 0 && y <= 0 && z >= 0) return { 6, 1 }; //blue
	if (x >= 0 && y >= 0 && z >= 0) return { 0, 7 }; //white
	if (x <= 0 && y <= 0 && z <= 0) return { 7, 0 }; //black
	if (x >= 0 && y <= 0 && z <= 0) return { 3, 4 }; //red
	if (x <= 0 && y >= 0 && z >= 0) return { 4, 3 }; //cyan
	if (x >= 0 && y <= 0 && z >= 0) return { 2, 5 }; //magenta
	if (x <= 0 && y >= 0 && z <= 0) return { 5, 2 }; //green
}

static pair<int, int> collisionDetection::projectedObbExtremes(const Vector3 & axis, const Obb & obb) {
	Matrix3 newBasis{ obb.right, obb.up, obb.forward };
	Vector3 newAxis = inverse(newBasis) * axis;
	return projectedAabbExtremes(newAxis);
}

static pair<float, float> collisionDetection::projectedFrustumExtremes(const Vector3 & axis, const Frustum & frustum) {
	const auto vertices = frustum.getPoints();
	float min = numeric_limits<float>::max(), max = -numeric_limits<float>::max();
	auto normalized = normalize(axis);

	for (const auto& vertex : vertices) {
		float projection = dot(axis, vertex);
		min = glm::min(min, projection);
		max = glm::max(max, projection);
	}

	return { min, max };
}


