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
	template<typename D>
	concept Distribution3d = requires(D distr, std::mt19937 rng) {
		{ distr(rng) } -> std::same_as<Vector3>;
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
	 * @brief Represents a random uniform 3-dimensional distribution.
	 */
	class Uniform3dDistribution {
	public:
		/**
		 * @brief Builds a uniform 3D random distribution where each component of the possibly generated vectors must be inside the specified ranges.
		 */
		Uniform3dDistribution(float minX, float maxX, float minY, float maxY, float minZ, float maxZ) 
			: distributionX{ minX, maxX }, distributionY{ minY, maxY }, distributionZ{ minZ, maxZ } {}

		/**
		 * @brief Returns a random 3D vector with each component inside the range specified during construction.
		 * Internally it calls @p std::uniform_real_distribution<>::(rng) 3 times.
		 */
		Vector3 operator()(std::mt19937& rng) {
			return Vector3{ distributionX(rng), distributionY(rng), distributionZ(rng) };
		}

	private:
		std::uniform_real_distribution<> distributionX;
		std::uniform_real_distribution<> distributionY;
		std::uniform_real_distribution<> distributionZ;
	};

	template<std::size_t N>
	struct ConvexHull {
	public:
		template<std::same_as<Vector3>... Vec3> //requires { sizeof...(Vec3) > 2; }
		ConvexHull(const Vec3&... vertices) : vertices{} {
			fillVertices(std::make_index_sequence<sizeof...(Vec3)>{}, vertices...);
		}

		/**
		 * @brief Returns the barycenter of the @p ConvexHull.
		 */
		Vector3 barycenter() const {
			Vector3 sum{ 0.0f, 0.0f, 0.0f };
			for (int i = 0; i < N; ++i) {
				sum += vertices[i];
			}
			return sum / N;
		}

		/**
		 * @brief Returns the normal to this @p ConvexHull.
		 */
		Vector3 normal() const {
			return glm::cross(vertices[1] - vertices[0], vertices[2] - vertices[0]);
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

		/**
		 * @brief Returns the array of the vertices of the @p ConvexHull..
		 */
		const std::array<Vector3, N>& getVertices() const {
			return vertices;
		}

	private:
		/**
		 * @brief Given a parameter pack of vertices, it fills the array pf vertices of this object.
		 */
		template<std::size_t... Is, std::same_as<Vector3>... Vec3>
		void fillVertices(std::index_sequence<Is...>, const Vec3&... vertices) {
			([this]<std::size_t I>(const Vec3& vec) { std::get<I>(this->vertices) = vec; }.template operator()<Is>(vertices), ...); //basically a compile time loop to fill the array with the variadic arg
			//break down of the statement above:
			// 1) Create a templated lambda: this lambda statically accesses the array at index I, and sets the content to vec
			// 2) The lambda is immediately called, but since it is templated and I mus be explicitly specified, we have to use the syntax: foo.template operator()<TemplateArg>(functionarg);
			// 3) The arguments to pass to the lambda are variadic template arguments (the first one is non-type), therefore we use the right fold syntax for the comma operator: (foo, ...);

			if constexpr (N > 3) checkCoplanar(); //if the hull has 3 vertices, they are always coplanar
		}

		/**
		 * @brief Checks if the vertices of the @p ConvexHull are coplanar. If they are not, it throws an exception.
		 *
		 */
		void checkCoplanar() const {
			if (vertices.size() == 3) return; //3 vertices are always coplanar

			Vector3 normal = glm::cross(vertices[0], vertices[1]);
			for (int i = 1; i < vertices.size() - 1; ++i) {
				Vector3 normalsDelta = glm::abs(glm::cross(vertices[i], vertices[i + 1]) - normal);
				if (normalsDelta.x > TOLERANCE || normalsDelta.y > TOLERANCE || normalsDelta.z > TOLERANCE) {
					throw std::logic_error{ "Points for this ConvexHull are not coplanar." };
				}
			}
		}


		std::array<Vector3, N> vertices;
	};
	template<std::same_as<Vector3>... Vec3> ConvexHull(const Vec3&... vertices) -> ConvexHull<sizeof...(Vec3)>; /**< @brief Deduction guide. */

	using Triangle = ConvexHull<3>;
	/**
	 * @brief Represents a triangle.
	 */
	template<>
	struct ConvexHull<3> {
		Vector3 v0, v1, v2;

		ConvexHull() = delete;
		ConvexHull(Vector3 v0, Vector3 v1, Vector3 v2) : v0{ v0 }, v1{ v1 }, v2{ v2 } {}

		/**
		 * @brief Returns the barycenter of the @p Triangle.
		 */
		Vector3 barycenter() const {
			return (v0 + v1 + v2) / 3.0f;
		}

		/**
		 * @brief Returns the normal to this @p Triangle.
		 */
		Vector3 normal() const {
			Vector3 e1 = v1 - v0; //first edge
			Vector3 e2 = v2 - v0; //second edge
			return glm::normalize(glm::cross(e1, e2));
		}

		/**
		 * @brief Returns the specified vertex of the @p Triangle.
		 * The index must be withing 0 and 3.
		 */
		Vector3& operator[](std::size_t i) {
			if (i == 0) return v0;
			if (i == 1) return v1;
			if (i == 2) return v2;
			throw std::out_of_range{ "Index must be 0, 1 or 2." };
		}

		/**
		 * @brief Returns the specified vertex of the @p Triangle.
		 * The index must be withing 0 and 3.
		 */
		Vector3 operator[](std::size_t i) const {
			if (i == 0) return v0;
			if (i == 1) return v1;
			if (i == 2) return v2;
			throw std::out_of_range{ "Index must be 0, 1 or 2." };
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
		Plane() : point{}, normal{ 0,0,1 } {}
		Plane(Vector3 point, Vector3 normal) : point{ point }, normal{ glm::normalize(normal) } {}

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
		Vector3 position;
		Vector3 direction;
		Vector3 up;

	public:
		Pov() : position{}, direction{ Vector3{1, 0, 0} }, up{ 0,1,0 } {}
		Pov(Vector3 position, Vector3 direction, Vector3 up = Vector3{ 0.0f, 1.0f, 0.0f }) : position{ position }, direction{ direction }, up{ up } {}
		Pov(Plane plane, Vector3 up = Vector3{ 0.0f, 1.0f, 0.0f }) : position{ plane.getPoint()}, direction{plane.getNormal()}, up{up} {}

		const Vector3& getPosition() const { return position; }
		void setPosition(Vector3 position) { this->position = position; }
		const Vector3& getDirection() const { return direction; }
		void setDirection(Vector3 direction) { this->direction = glm::normalize(direction); }		
		const Vector3& getUp() const { return up; }
		void setUp(Vector3 up) { this->up = glm::normalize(up); }
	};

	struct Ray {
	private:
		Vector3 origin;
		Vector3 direction;

	public:
		Ray() : origin{}, direction{ Vector3{1, 0, 0} } {}
		Ray(Vector3 origin, Vector3 direction) : origin{ origin }, direction{ direction } {}
		
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
