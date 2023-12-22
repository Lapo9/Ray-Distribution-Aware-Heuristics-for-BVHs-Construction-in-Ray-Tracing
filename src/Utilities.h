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

namespace pah {
	#define PerNodeActionType void(GlobalObject&, const Bvh::Node&, const Bvh&, int currLvl, json&)
	#define FinalActionType void(GlobalObject&, const Bvh&, json&)

	using Vector2 = glm::vec2;
	using Vector3 = glm::vec3;
	using Vector4 = glm::vec4;
	using Matrix3 = glm::mat3;
	using Matrix4 = glm::mat4;

	template<typename D>
	concept Distribution3d = requires(D distr, std::mt19937 rng) {
		{ distr(rng) } -> std::same_as<Vector3>;
	};

	template<typename P>
	concept PointLike = requires(P point) {
		{ point.x } -> std::convertible_to<float>; //must have float .x
		{ point.y } -> std::convertible_to<float>; //must have float .y
		(not requires { point.z; }) || requires{ { point.z } -> std::convertible_to<float>; }; //if it has .z it must be float
		(not requires { point.w; }) || requires{ { point.z } -> std::convertible_to<float>;
											     { point.w } -> std::convertible_to<float>; }; //if it has .w it must also have .z, and they both must be float
	};

	class Uniform3dDistribution {
	public:
		Uniform3dDistribution(float minX, float maxX, float minY, float maxY, float minZ, float maxZ) 
			: distributionX{ minX, maxX }, distributionY{ minY, maxY }, distributionZ{ minZ, maxZ } {}

		Vector3 operator()(std::mt19937& rng) {
			return Vector3{ distributionX(rng), distributionY(rng), distributionZ(rng) };
		}

	private:
		std::uniform_real_distribution<> distributionX;
		std::uniform_real_distribution<> distributionY;
		std::uniform_real_distribution<> distributionZ;
	};


	struct Triangle {
		Vector3 v1, v2, v3;

		Triangle() = delete;
		Triangle(Vector3 v1, Vector3 v2, Vector3 v3) : v1{ v1 }, v2{ v2 }, v3{ v3 } {}

		Vector3 center() const {
			return (v1 + v2 + v3) / 3.0f;
		}

		template<Distribution3d D, Distribution3d D2>  
		static Triangle random(std::mt19937& rng, D& firstVertexDistribution, D2& otherVerticesDistributions) {
			Vector3 v1 = firstVertexDistribution(rng);
			Vector3 v2 = v1 + otherVerticesDistributions(rng);
			Vector3 v3 = v1 + otherVerticesDistributions(rng);
			return Triangle{ v1, v2, v3 };
		}

		template<Distribution3d D, Distribution3d D2>
		static std::vector<Triangle> generateRandom(int qty, std::mt19937& rng, D& firstVertexDistribution, D2& otherVerticesDistributions) {
			std::vector<Triangle> triangles;
			for (int i = 0; i < qty; ++i) {
				triangles.emplace_back(random(rng, firstVertexDistribution, otherVerticesDistributions));
			}
			return triangles;
		}
	};

	enum class Axis {
		X, Y, Z, None
	};

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


		class TimeLogger {
		public:
			template<std::convertible_to<std::function<void(std::chrono::duration<float, std::milli>)>>... FinalActions>
			TimeLogger(const FinalActions&... finalActions) : startTime{ std::chrono::high_resolution_clock::now() } {
				(this->finalActions.push_back(finalActions), ...);
			}
			
			TimeLogger(const TimeLogger&) = default;
			TimeLogger& operator=(const TimeLogger&) = default;
			TimeLogger(TimeLogger&&) = default;
			TimeLogger& operator=(TimeLogger&&) = default;
			
			~TimeLogger() {
				if(runLogOnDestruction) log();
			}

			void log() {
				auto duration = std::chrono::high_resolution_clock::now() - startTime;
				assert((duration.count() > 0, "Recorded duration <= 0"));
				for (const auto& finalAction : finalActions) {
					finalAction(duration);
				}
			}

			void stop() {
				log();
				runLogOnDestruction = false;
			}

			//possible final actions

			static void print(std::chrono::duration<float, std::milli> duration, std::string name) {
				std::cout << name << " " << duration << std::endl;
			}

			static void json(std::chrono::duration<float, std::milli> duration, nlohmann::json& jsonOut, std::string label) {
				jsonOut[label] = duration.count();
			}

		private:
			std::chrono::time_point<std::chrono::high_resolution_clock> startTime;
			std::vector<std::function<void(std::chrono::duration<float, std::milli> duration)>> finalActions;
			bool runLogOnDestruction = true;
		};
	}
}
