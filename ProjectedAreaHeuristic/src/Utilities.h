#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <vector>
#include <array>
#include <random>
#include <limits>
#include <iostream>
#include <stdexcept>
#include <concepts>
#include <fstream>
#include <ranges>

#include  "../libs/json.hpp"

#include "settings.h"

namespace pah {
	#define PerNodeActionType void(GlobalObject&, const Bvh::Node&, const Bvh&, int currLvl, json&)
	#define FinalActionType void(GlobalObject&, const Bvh&, json&)

	using Vector2 = glm::vec2;
	using Vector3 = glm::vec3;
	using Vector4 = glm::vec4;
	using Matrix3 = glm::mat3;
	using Matrix4 = glm::mat4;
	using DurationMs = std::chrono::duration<float, std::milli>;

	/**
	 * @brief Represents a random distribution in 3 dimensions. @p distribution(rng) must return a @p Vector3.
	 */
	template<typename D, typename Rng = std::mt19937>
	concept Distribution3d = std::uniform_random_bit_generator<Rng> &&
		requires(D distr, Rng rng) {
			{ distr(rng) } -> std::same_as<Vector3>;
	};

	/**
	 * @brief Represents a random distribution in 2 dimensions. @p distribution(rng) must return a @p Vector2.
	 */
	template<typename D, typename Rng = std::mt19937>
	concept Distribution2d = std::uniform_random_bit_generator<Rng> &&
		requires(D distr, Rng rng) {
			{ distr(rng) } -> std::same_as<Vector2>;
	};

	/**
	 * @brief Represents anything that looks like a point (i.e. it is possible to access it like @p object.x/y[/z/w].
	 */
	template<typename P>
	concept PointLike = requires(P point) {
		{ point.x } -> std::convertible_to<float>; //must have float .x
		{ point.y } -> std::convertible_to<float>; //must have float .y
		(not requires { point.z; }) || requires{ { point.z } -> std::convertible_to<float>; }; //if it has .z it must be float
		(not requires { point.w; }) || requires{ { point.z } -> std::convertible_to<float>;
											     { point.w } -> std::convertible_to<float>; }; //if it has .w it must also have .z, and they both must be float
	};


	/**
	 * @brief Represents any polygonal convex shape in 2 dimensions.
	 */
	struct ConvexHull2d {
	public:
		template<std::same_as<Vector2> Vec2, std::same_as<Vector2>... Vec2s>
		ConvexHull2d(const Vec2& v0, const Vec2& v1, const Vec2& v2, const Vec2s&... vertices) : vertices{} {
			this->vertices.push_back(v0);
			this->vertices.push_back(v1);
			this->vertices.push_back(v2);
			(this->vertices.push_back(vertices), ...);
		}
		
		ConvexHull2d() {}
		
		ConvexHull2d(const std::vector<Vector2>& vertices) : vertices{ vertices } {
			if (size() < 3 && size() != 0) throw std::logic_error{ "A ConvexHull2d must have at least 3 vertices, or 0. This one has " + size() };
		}

		/**
		 * @brief Returns the barycenter of the @p ConvexHull.
		 */
		Vector2 barycenter() const {
			Vector2 sum{ 0.0f, 0.0f };
			for (int i = 0; i < size(); ++i) {
				sum += vertices[i];
			}
			return sum / (float)size();
		}

		/**
		 * @brief Returns the specified vertex of the @p ConvexHull2d.
		 */
		Vector2& operator[](std::size_t i) {
			return vertices[i];
		}

		/**
		 * @brief Returns the specified vertex of the @p ConvexHull2d.
		 */
		Vector2 operator[](std::size_t i) const {
			return vertices[i];
		}

		std::size_t size() const {
			return vertices.size();
		}

		/**
		 * @brief Returns the array of the vertices of the @p ConvexHull2d.
		 */
		const std::vector<Vector2>& getVertices() const {
			return vertices;
		}

		/**
		 * @brief Returns whether a point is inside the hull.
		 */
		bool isPointInside(const Vector2& P) const {
			if (size() == 0) return false;

			// returns whether the cross product of 2d vectors is positive
			auto cross2d = [](const Vector2& lhs, const Vector2& rhs) -> float { return lhs.x * rhs.y - rhs.x * lhs.y; };

			// we compute the cross product between the first edge and the vector connecting the first vertex of the edge and our point.
			// this will be > 0 if the point is "to the left" and < 0 if it is "to the right" of the edge.
			// we do the same with all the edges: the point is inside iff it is to the left/right of all edges.
			int M = size();
			Vector2 edge = vertices[0] - vertices[M - 1];
			Vector2 p = P - vertices[M - 1];
			float signum = cross2d(edge, p);
			for (std::size_t i = 0; i < M - 1; ++i) {
				edge = vertices[i + 1] - vertices[i]; //vector representing a triangle edge
				p = P - vertices[i]; //vector from the vertex of the triangle to the intersection point
				float cross = cross2d(edge, p);
				signum = signum == 0 ? cross : signum; //if the signum we calculated before the loop was (very unfortunately) 0, we get the "base" signum here
				if (cross * signum < 0) return false;
			}

			return true;
		}

		/**
		 * @brief Returns the area of the hull.
		 */
		float computeArea() const {
			if (size() < 3) return 0;
			float area = 0.0f;
			//use the shoelace formula to comute the area
			for (int i = 0; i < vertices.size(); ++i) {
				int iNext = i + 1 == vertices.size() ? 0 : i + 1;
				area += vertices[i].x * vertices[iNext].y - vertices[i].y * vertices[iNext].x;
			}
			return abs(area / 2.f); //we can probably just return area, since we use the area to calculate the hit probability, therefore a ratio between 2 areas
		}

		/**
		 * @brief Returns the hull obtained by intersecting 2 @p ConvexHull2d
		 */
		friend ConvexHull2d overlappingHull(const ConvexHull2d& h1, const ConvexHull2d& h2) {
			if (h1.size() == 0 || h2.size() == 0) return ConvexHull2d{};
			std::vector<Vector2> vertices{};

			// find all the points of a hull, internal to the other hull
			for (int i = 0; i < h1.size(); ++i) {
				if (h2.isPointInside(h1[i])) {
					vertices.push_back(h1[i]);
				}
			}
			for (int i = 0; i < h2.size(); ++i) {
				if (h1.isPointInside(h2[i])) {
					vertices.push_back(h2[i]);
				}
			}

			// function to find the intersection point between 2 segments (if present)
			auto segmentsIntersection = [](const Vector2& a1, const Vector2& a2, const Vector2& b1, const Vector2& b2) -> std::pair<bool, Vector2> {
				//Ax+By = C
				float A1 = a2.y - a1.y; float B1 = a1.x - a2.x; float C1 = A1 * a1.x + B1 * a1.y;
				float A2 = b2.y - b1.y; float B2 = b1.x - b2.x; float C2 = A2 * b1.x + B2 * b1.y;
				float det = A1 * B2 - A2 * B1;
				if (det == 0) return { false, Vector2{0,0} };

				//intersection point coords
				Vector2 res{ (B2 * C1 - B1 * C2) / det, (A1 * C2 - A2 * C1) / det };

				//is the intersection point inside the segment?
				bool onSegmA = fmin(a1.x, a2.x) <= res.x + TOLERANCE && fmax(a1.x, a2.x) >= res.x - TOLERANCE && fmin(a1.y, a2.y) <= res.y + TOLERANCE && fmax(a1.y, a2.y) >= res.y - TOLERANCE;
				bool onSegmB = fmin(b1.x, b2.x) <= res.x + TOLERANCE && fmax(b1.x, b2.x) >= res.x - TOLERANCE && fmin(b1.y, b2.y) <= res.y + TOLERANCE && fmax(b1.y, b2.y) >= res.y - TOLERANCE;
				return { onSegmA && onSegmB, res };
				};

			// find all intersection points of the edges
			for (int i = 0; i < h1.size(); ++i) {
				for (int j = 0; j < h2.size(); ++j) {
					auto [intersect, point] = segmentsIntersection(h1[i], h1[(i + 1) % h1.size()], h2[j], h2[(j + 1) % h2.size()]);
					if (intersect) vertices.push_back(point);
				}
			}

			if (vertices.size() < 3) return ConvexHull2d{};

			// sort all the points in counterclockwise order (find a point inside the overlap hull and sort by atan2)
			std::ranges::sort(vertices, [center = ConvexHull2d{ vertices }.barycenter()](const Vector2& a, const Vector2& b) {return atan2(a.y - center.y, a.x - center.x) >= atan2(b.y - center.y, b.x - center.x); });
			return ConvexHull2d{ vertices };
		}

		/**
		 * @brief Returns the amount of overlapping area between 2 convex hulls.
		 */
		friend float overlappingArea(const ConvexHull2d& h1, const ConvexHull2d& h2) {
			return overlappingHull(h1, h2).computeArea();
		}

	private:

		std::vector<Vector2> vertices;
	};


	/**
	 * @brief Represents any polygonal convex shape.
	 */
	struct ConvexHull3d {
	public:
		template<std::same_as<Vector3>... Vec3> //requires { sizeof...(Vec3) > 2; }
		ConvexHull3d(const Vec3&... vertices) : vertices{} {
			(this->vertices.push_back(vertices), ...);
			checkCoplanar();
		}

		ConvexHull3d(const std::vector<Vector3>& vertices) : vertices{ vertices } {}

		/**
		 * @brief Returns the barycenter of the @p ConvexHull.
		 */
		Vector3 barycenter() const {
			Vector3 sum{ 0.0f, 0.0f, 0.0f };
			for (int i = 0; i < size(); ++i) {
				sum += vertices[i];
			}
			return sum / (float)size();
		}

		/**
		 * @brief Returns the normal to this @p ConvexHull.
		 */
		Vector3 normal() const {
			return glm::normalize(glm::cross(vertices[1] - vertices[0], vertices[2] - vertices[0]));
		}

		/**
		 * @brief Returns the specified vertex of the @p ConvexHull.
		 * The index must be withing 0 and @p N.
		 */
		Vector3& operator[](std::size_t i) {
			return vertices[i];
		}

		/**
		 * @brief Returns the specified vertex of the @p ConvexHull.
		 * The index must be withing 0 and @p N.
		 */
		Vector3 operator[](std::size_t i) const {
			return vertices[i];
		}

		std::size_t size() const {
			return vertices.size();
		}

		/**
		 * @brief Returns the array of the vertices of the @p ConvexHull..
		 */
		const std::vector<Vector3>& getVertices() const {
			return vertices;
		}

		/**
		 * @brief Returns whether a point is inside the hull.
		 */
		bool isPointInside(const Vector3& P) const {
			// The point is inside iff it is "to the left" of all sides taken in counter clockwise order (or "to the right" in counterclockwise order)
			// We determine whether the point is to the left or right of the first edge, by calculating the cross product between the edge vector and the vector connecting the first vertex of the edge to our point.
			// Then we try the sam ewith all the edges, and the cross product result must point in the same direction as the first one (therefore the dot product should be > 0)
			int M = size();
			Vector3 edge = vertices[0] - vertices[M - 1];
			Vector3 p = P - vertices[M - 1];
			auto N = cross(edge, p);
			for (std::size_t i = 0; i < M - 1; ++i) {
				edge = vertices[i + 1] - vertices[i]; //vector representing a triangle edge
				p = P - vertices[i]; //vector from the vertex of the triangle to the intersection point
				if (dot(N, cross(edge, p)) <= 0) return false;
			}
			return true;
		}
		
		/**
		 * @brief Returns the area of the hull..
		 */
		float computeArea() const {
			float area = 0.0f;
			//use the shoelace formula to comute the area
			for (int i = 0; i < vertices.size(); ++i) {
				int iNext = i + 1 == vertices.size() ? 0 : i + 1;
				area += vertices[i].x * vertices[iNext].y - vertices[i].y * vertices[iNext].x;
			}
			return abs(area / 2.f); //we can probably just return area, since we use the area to calculate the hit probability, therefore a ratio between 2 areas
		}

	private:
		/**
		 * @brief Checks if the vertices of the @p ConvexHull are coplanar. If they are not, it throws an exception.
		 *
		 */
		void checkCoplanar() const {
			using namespace glm;
			if (vertices.size() == 3) return; //3 vertices are always coplanar

			Vector3 normal = normalize(cross(vertices[0] - vertices[1], vertices[1]-vertices[2]));
			for (int i = 3; i < vertices.size(); ++i) {
				Vector3 currentNormal = normalize(cross(vertices[i - 2] - vertices[i - 1], vertices[i - 1] - vertices[i]));
				Vector3 normalsDelta = min(abs(currentNormal - normal), abs(currentNormal + normal)); //we are interested in the direction, not the "verse" of the direction
				if (normalsDelta.x > TOLERANCE || normalsDelta.y > TOLERANCE || normalsDelta.z > TOLERANCE) {
					throw std::logic_error{ "Points for this ConvexHull are not coplanar." };
				}
			}
		}

		std::vector<Vector3> vertices;
	};


	/**
	 * @brief Represents a triangle.
	 */
	struct Triangle {
		//Vector3 v0, v1, v2;

		Triangle() = delete;
		Triangle(Vector3 v0, Vector3 v1, Vector3 v2) : triangle{ v0,v1,v2 } {}

		/**
		 * @brief Returns the barycenter of the @p Triangle.
		 */
		Vector3 barycenter() const {
			return triangle.barycenter();
		}

		/**
		 * @brief Returns the normal to this @p Triangle.
		 */
		Vector3 normal() const {
			triangle.normal();
		}

		/**
		 * @brief Returns the specified vertex of the @p Triangle.
		 * The index must be withing 0 and 3.
		 */
		Vector3& operator[](std::size_t i) {
			return triangle[i];
		}

		/**
		 * @brief Returns the specified vertex of the @p Triangle.
		 * The index must be withing 0 and 3.
		 */
		Vector3 operator[](std::size_t i) const {
			return triangle[i];
		}

		/**
		 * @brief Returns whether a point is inside the triangle.
		 */
		bool isPointInside(const Vector3& P) const {
			return triangle.isPointInside(P);
		}

		/**
		 * @brief Returns the area of the triangle.
		 */
		float computeArea() const {
			return triangle.computeArea();
		}

		/**
		 * @brief Returns the underlying convex hull.
		 * 
		 * @return .
		 */
		const ConvexHull3d& operator*() const {
			return triangle;
		}


		/**
		 * @brief Generates a random triangle.
		 */
		template<Distribution3d D, Distribution3d D2>  
		static Triangle random(std::mt19937& rng, D& firstVertexDistribution, D2& otherVerticesDistributions) {
			Vector3 v0 = firstVertexDistribution(rng);
			Vector3 v1 = v0 + otherVerticesDistributions(rng);
			Vector3 v2 = v1 + otherVerticesDistributions(rng);
			return Triangle{ v0, v1, v2 }; 
		}

		/**
		 * @brief Generate a list of random triangles.
		 */
		template<Distribution3d D, Distribution3d D2>
		static std::vector<Triangle> generateRandom(int qty, std::mt19937& rng, D& firstVertexDistribution, D2& otherVerticesDistributions) {
			std::vector<Triangle> triangles;
			for (int i = 0; i < qty; ++i) {
				triangles.emplace_back(random(rng, firstVertexDistribution, otherVerticesDistributions));
			}
			return triangles;
		}

		/**
		 * @brief Returns an array of triangles from a .obj waveform file.
		 * We assume that all the vertices lines are placed before the triangle lines.
		 */
		static std::vector<Triangle> fromObj(const std::string& filePath) {
			using namespace std;

			std::ifstream objFile{ filePath };
			vector<Triangle> triangles;
			vector<Vector3> vertices;

			string lineStr;
			while (getline(objFile, lineStr)) {
				string lineType; //f: triangle, v: vertex (vn: normal and vt: UVs but we care only about t and v)
				istringstream lineStream{ lineStr };
				lineStream >> lineType;

				// vertex
				if (lineType == "v") {
					float x, y, z;
					lineStream >> x >> y >> z;
					vertices.emplace_back(x, y, z);
				}
			}

			// go to the start of the file again
			objFile.clear();
			objFile.seekg(0);
			while (getline(objFile, lineStr)) {
				string lineType; //f: triangle, v: vertex (vn: normal and vt: UVs but we care only about t and v)
				istringstream lineStream{ lineStr };
				lineStream >> lineType;

				// a triangle line has this form: f 1/2/3 4/5/6 7/8/9 where vertexIndex/uvIndex/normalIndex
				if (lineType == "f") {
					string indicesStr;
					Triangle triangle{ Vector3{}, Vector3{}, Vector3{} };

					// here we get the first string (e.g. 1/2/3, then 4/5/6, then 7/8/9)
					for (int i = 0; i < 3 && lineStream >> indicesStr; ++i) {
						istringstream indicesStream{ indicesStr };
						string vertexIndexStr;
						getline(indicesStream, vertexIndexStr, '/'); // we don't care about uv and normal, only the vertex index

						int vertexIndex = atoi(vertexIndexStr.c_str());
						vertexIndex = (vertexIndex >= 0 ? vertexIndex : vertices.size() + vertexIndex);

						triangle[i] = vertices[vertexIndex-1]; // add the vertex to the triangle
					}
					triangles.emplace_back(triangle);
				}
			}
			return triangles;
		}

		private:
			ConvexHull3d triangle;
	};


	/**
	 * @brief Represents the canonical 3D axes.
	 */
	enum class Axis {
		X, Y, Z, None
	};


	/**
	 * @brief Represents a plane.
	 */
	struct Plane {
	private: 
		Vector3 normal;
		Vector3 point;

	public:
		float width;
		float height;

		Plane() : point{}, normal{ 0,0,1 } {}
		Plane(Vector3 point, Vector3 normal, float width = 0, float height = 0) : point{ point }, normal{ glm::normalize(normal) }, width{ width }, height{ height } {}

		const Vector3& getPoint() const { return point; }
		void setPoint(Vector3 point) { this->point = point; }
		const Vector3& getNormal() const { return normal; }
		void setNormal(Vector3 normal) { this->normal = glm::normalize(normal); }
	};


	/**
	 * @brief Represents a 3D point of view.
	 */
	struct Pov {
	private:
		Vector3 direction;
		Vector3 up;
	public:
		Vector3 position;
		float fovX;
		float fovY;


		Pov() : position{}, direction{ Vector3{1, 0, 0} }, up{ 0,1,0 }, fovX{ glm::radians(90.0f) }, fovY{ glm::radians(90.0f) } {}
		Pov(Vector3 position, Vector3 direction, float fovXdegrees, float fovYdegrees, Vector3 up = Vector3{ 0.0f, 1.0f, 0.0f }) : 
			position{ position }, direction{ glm::normalize(direction) }, up{ glm::normalize(up) }, fovX{ fovXdegrees }, fovY{ fovYdegrees } {
		}

		const Vector3& getDirection() const { return direction; }
		void setDirection(Vector3 direction) { this->direction = glm::normalize(direction); }		
		const Vector3& getUp() const { return up; }
		void setUp(Vector3 up) { this->up = glm::normalize(up); }
	};


	/**
	 * @brief Represents a ray in 3D.
	 */
	struct Ray {
	private:
		Vector3 origin;
		Vector3 direction;

	public:
		Ray() : origin{}, direction{ Vector3{1, 0, 0} } {}
		Ray(Vector3 origin, Vector3 direction) : origin{ origin }, direction{ glm::normalize(direction) } {}
		
		const Vector3& getOrigin() const { return origin; }
		void setOrigin(Vector3 origin) { this->origin = origin; }
		const Vector3& getDirection() const { return direction; }
		void setDirection(Vector3 direction) { this->direction = glm::normalize(direction); }
	};


	namespace utilities {

		/**
		 * @brief Given a vector and an axis, returns the corresponding component of the vector.
		 */
		static float at(const Vector3& vector, Axis axis) {
			if (axis == Axis::X) return vector.x;
			if (axis == Axis::Y) return vector.y;
			if (axis == Axis::Z) return vector.z;
			throw std::invalid_argument{ "Cannot use pah::Axis::None as argument" };
		}

		/**
		 * @brief Given 2 axis, it returns the remaining one.
		 */
		static Axis third(Axis a1, Axis a2) {
			if (a1 == Axis::X && a2 == Axis::Y || a1 == Axis::Y && a2 == Axis::X) return Axis::Z;
			if (a1 == Axis::X && a2 == Axis::Z || a1 == Axis::Z && a2 == Axis::X) return Axis::Y;
			if (a1 == Axis::Y && a2 == Axis::Z || a1 == Axis::Z && a2 == Axis::Y) return Axis::X;
			throw std::invalid_argument{ "Cannot use pah::Axis::None as argument" };
		}

		/**
		 * @brief Given an axis it returns the remaining 2.
		 */
		static std::tuple<Axis, Axis> other2(Axis axis) {
			if (axis == Axis::X) return { Axis::Y, Axis::Z };
			if (axis == Axis::Y) return { Axis::X, Axis::Z };
			if (axis == Axis::Z) return { Axis::X, Axis::Y };
			throw std::invalid_argument{ "Cannot use pah::Axis::None as argument" };
		}

		/**
		 * @brief Returns a right-handed coordinate system where the argument points to the z direction.
		 */
		static std::tuple<Vector3, Vector3, Vector3> rightHandCoordinatesSystem(Vector3 forward, const Vector3& up = Vector3{ 0.0f, 1.0f, 0.0f }) {
			using namespace glm;

			forward = normalize(forward);
			Vector3 right = normalize((forward == Vector3{ 0.0f, 1.0f, 0.0f } || forward == Vector3{ 0.0f, -1.0f, 0.0f }) ? Vector3{ 1.0f, 0.0f, 0.0f } : cross(up, forward));
			Vector3 upDir = normalize(cross(forward, right));
			return { right, upDir, forward };
		}

		/**
		 * @brief Returns a left-handed coordinate system where the argument points to the z direction.
		 */
		static std::tuple<Vector3, Vector3, Vector3> leftHandCoordinatesSystem(Vector3 forward, const Vector3& up = Vector3{ 0.0f, 1.0f, 0.0f }) {
			using namespace glm;

			forward = normalize(forward);
			Vector3 right = normalize((forward == Vector3{ 0.0f, 1.0f, 0.0f } || forward == Vector3{ 0.0f, -1.0f, 0.0f }) ? Vector3{ -1.0f, 0.0f, 0.0f } : cross(forward, up));
			Vector3 upDir = normalize(glm::cross(right, forward));
			return { right, upDir, forward };
		}

		/**
		 * @brief Class that can be used to measure the elapsed time between 2 points in a single thread of the code.
		 */
		class TimeLogger {
		public:
			/**
			 * @brief Creates a @p TimeLogger. The specified action will be executed when the @TimeLogger is stopped or destroyed.
			 * 
			 * @param ...finalActions .
			 */
			template<std::convertible_to<std::function<void(std::chrono::duration<float, std::milli>)>>... FinalActions>
			TimeLogger(const FinalActions&... finalActions) : startTime{ std::chrono::high_resolution_clock::now() } {
				(this->finalActions.push_back(finalActions), ...);
			}
			
			TimeLogger(const TimeLogger&) = default;
			TimeLogger& operator=(const TimeLogger&) = default;
			TimeLogger(TimeLogger&&) = default;
			TimeLogger& operator=(TimeLogger&&) = default;
			
			/**
			 * @brief If the @p TimeLogger was not stopped before, executes the final action and destroys the @p TimeLogger. Else simply destroys it.
			 * 
			 */
			~TimeLogger() {
				if(!stopped) log();
			}

			/**
			 * @brief Executes the final action, but the @p TimeLogger still continues to run.
			 */
			void log() {
				std::chrono::duration<float, std::milli> duration = std::chrono::high_resolution_clock::now() - startTime;
				resume(); //if the timer was paused, we resume it, so that we get the correct paused time
				duration -= pausedTime; //real duration = time passed from start to finish - paused time
				assert((duration.count() > 0, "Recorded duration <= 0"));
				for (const auto& finalAction : finalActions) {
					finalAction(duration);
				}
			}

			/**
			 * @brief Executes the final action, and the @p TimeLogger stops. Can be called only once.
			 * 
			 */
			void stop() {
				throwIfStopped();
				log();
				stopped = true;
			}

			/**
			 * @brief Pauses the timer. If the timer was already paused, nothing happens.
			 */
			void pause() {
				throwIfStopped();
				if (!paused) {
					lastPauseTime = std::chrono::high_resolution_clock::now();
					paused = true;
				}
			}

			/**
			 * @brief Resumes the timer. If the timer was already running, nothing happens.
			 */
			void resume() {
				throwIfStopped();
				if (paused) {
					pausedTime += std::chrono::high_resolution_clock::now() - lastPauseTime;
					paused = false;
				}
			}


			//possible final actions

			/**
			 * @brief Prints the measured time.
			 */
			static void print(std::chrono::duration<float, std::milli> duration, std::string name) {
				std::cout << name << " " << duration << std::endl;
			}

			/**
			 * @brief Prints the measured time to a JSON object.
			 */
			static void json(std::chrono::duration<float, std::milli> duration, nlohmann::json& jsonOut, std::string label) {
				jsonOut[label] = duration.count();
			}

		private:
			/**
			 * @brief Throws an exception if the timer is stopped.
			 */
			void throwIfStopped() const {
				if (stopped) { 
					throw std::exception("Cannot perform any action on an already stopped TimeLogger."); 
				}
			}

			std::chrono::time_point<std::chrono::high_resolution_clock> startTime;
			std::chrono::time_point<std::chrono::high_resolution_clock> lastPauseTime;
			std::chrono::duration<float, std::milli> pausedTime; //the time the timer has been paused
			std::vector<std::function<void(std::chrono::duration<float, std::milli> duration)>> finalActions;
			bool stopped = false;
			bool paused = false;
		};
	}
}
