#pragma once

#include "Utilities.h"

namespace pah::distributions {
	/**
	 * @brief Represents the distribution specified by a probability density function (PDF).
	 * The PDF by definition must always be positive and integrate to 1.
	 * The cumulative density function (CDF) is the integral of the PDF.
	 */
	class InvertedCdfDistribution {
	public:

		/**
		 * @brief Given the inverted cumulative density function (CDF) of a probability density function (PDF), it returns samples with the distribution specified by the PDF.
		 *
		 * @param inverseCdf The inverse of the CDF. The CDF is the integral of the PDF. The PDF must integrate to 1 and be positive.
		 */
		InvertedCdfDistribution(std::function<float(float)> inverseCdf) : uniformDistribution{}, inverseCdf{ inverseCdf } {}

		float operator()(std::uniform_random_bit_generator auto& rng) {
			float uniformSample = uniformDistribution(rng);
			return inverseCdf(uniformSample);
		}

	private:
		std::uniform_real_distribution<> uniformDistribution;
		std::function<float(float)> inverseCdf;
	};


	/**
	 * @brief Represents a uniform distribution over the surface of a sphere cap.
	 * Visualilzation at: https://www.geogebra.org/m/sbtknqat.
	 * Explanation of the sampling method at: https://www.desmos.com/calculator/sg5bz4pft8
	 */
	class UniformSphereCapDistribution {
	public:
		/**
		 * @brief Builds a distribution to sample points on the surface of a sphere cap.
		 *
		 * @param center The center of the sphere.
		 * @param orientation The orientation of the apex of the sphere cap.
		 * @param radius The radius of the sphere.
		 * @param halfCapAngle The polar angle of the sphere cap.
		 */
		UniformSphereCapDistribution(float halfCapAngle, Vector3 orientation = { 0,1,0 }, Vector3 center = { 0,0,0 }, float radius = 1) :
			rollDistribution{ 0, 2 * glm::pi<float>() },
			yawDistribution{ [R = radius, pi = glm::pi<float>(), k = radius * 2.85f * glm::pow(halfCapAngle, 1.81f)](float x) {
				float a = 2.0f * pi * R / k;
				return glm::acos((a - x) / a); //Explanation here: https://www.desmos.com/calculator/sg5bz4pft8
			} },
			center{ center }, orientation{ glm::normalize(orientation) }, radius{ radius }, halfCapAngle{ halfCapAngle },
			right{ std::get<0>(utilities::rightHandCoordinatesSystem(this->orientation)) } {
			if (radius <= 0) {
				throw std::logic_error{ "The radius of a UniformSphereCapDistribution must be > 0" };
			}
		}

		/**
		 * @brief Returns a uniformly random point on the surface of a sphere cap of the specified radius and angle.
		 */
		Vector3 operator()(std::uniform_random_bit_generator auto& rng) {
			if (halfCapAngle == 0.0f) return orientation;

			//extract the 2 angles
			float yaw = yawDistribution(rng);
			float roll = rollDistribution(rng);

			Vector3 res = orientation;
			//create rotation matrices
			const auto& yawRotation = glm::rotate(Matrix4{1.0f}, yaw, right);
			const auto& yawRollRotation = glm::rotate(yawRotation, roll, orientation); //the first argument is post multiplied
			res = yawRollRotation * Vector4{ res,1.0f }; //rotate the vector that points to the required direction
			return center + res * radius; //compute the final point

			//TODO yet to be tested
		}

	private:
		std::uniform_real_distribution<> rollDistribution;
		InvertedCdfDistribution yawDistribution;
		float radius;
		float halfCapAngle;
		Vector3 center;
		Vector3 orientation;
		Vector3 right;
	};


	/**
	 * @brief Represents a uniform distribution over the surface of a square sphere cap.
	 * Visualization at: https://www.geogebra.org/calculator/bzswwgvj.
	 */
	class UniformSquareSphereCapDistribution {
	public:
		/**
		 * @brief Builds a distribution to sample points on the surface of a square sphere cap.
		 *
		 * @param center The center of the sphere.
		 * @param orientation The orientation of the apex of the sphere cap.
		 * @param radius The radius of the sphere.
		 */
		UniformSquareSphereCapDistribution(float yawAngle, float pitchAngle, Vector3 orientation = { 0,1,0 }, Vector3 center = { 0,0,0 }, float radius = 1) :
			yawDistribution{ -yawAngle, yawAngle }, pitchDistribution{ -pitchAngle, pitchAngle }, yawAngle{ yawAngle }, pitchAngle{ pitchAngle },
			radius{ radius }, center{ center }, orientation{ glm::normalize(orientation) },
			right{ std::get<0>(utilities::rightHandCoordinatesSystem(this->orientation)) },
			up{ std::get<1>(utilities::rightHandCoordinatesSystem(this->orientation)) } {
		}

		/**
		 * @brief Returns a uniformly random point on the surface of a square sphere cap of the specified radius and angle.
		 */
		Vector3 operator()(std::uniform_random_bit_generator auto& rng) {
			//extract the 2 angles
			float yaw = yawDistribution(rng);
			float pitch = pitchDistribution(rng);

			Vector3 res = orientation;
			const auto& yawRotation = glm::rotate(Matrix4{1.0f}, yaw, right);
			const auto& yawPitchRotation = glm::rotate(yawRotation, pitch, up);
			res = yawPitchRotation * Vector4{ res,1.0f }; //rotate the vector that points to the required direction
			return center + res * radius; //compute the final point

			//TODO yet to be tested
		}

	private:
		std::uniform_real_distribution<> yawDistribution;
		std::uniform_real_distribution<> pitchDistribution;
		float yawAngle;
		float pitchAngle;
		float radius;
		Vector3 center;
		Vector3 orientation;
		Vector3 right;
		Vector3 up;
	};


	/**
	 * @brief Represents a random uniform 3-dimensional distribution.
	 */
	class UniformBoxDistribution {
	public:
		/**
		 * @brief Builds a uniform 3D random distribution where each component of the possibly generated vectors must be inside the specified ranges.
		 */
		UniformBoxDistribution(float minX, float maxX, float minY, float maxY, float minZ, float maxZ)
			: distributionX{ minX, maxX }, distributionY{ minY, maxY }, distributionZ{ minZ, maxZ } {
		}

		/**
		 * @brief Returns a random 3D vector with each component inside the range specified during construction.
		 * Internally it calls @p std::uniform_real_distribution<>::(rng) 3 times.
		 */
		Vector3 operator()(std::uniform_random_bit_generator auto& rng) {
			return Vector3{ distributionX(rng), distributionY(rng), distributionZ(rng) };
		}

	private:
		std::uniform_real_distribution<> distributionX;
		std::uniform_real_distribution<> distributionY;
		std::uniform_real_distribution<> distributionZ;
	};


	/**
	 * @brief Represents a random uniform 2-dimensional distribution.
	 */
	class UniformRectangleDistribution {
	public:
		/**
		 * @brief Builds a uniform 2D random distribution where each component of the possibly generated vectors must be inside the specified ranges.
		 */
		UniformRectangleDistribution(float minX, float maxX, float minY, float maxY)
			: distributionX{ minX, maxX }, distributionY{ minY, maxY } {
		}

		/**
		 * @brief Returns a random 2D vector with each component inside the range specified during construction.
		 * Internally it calls @p std::uniform_real_distribution<>::(rng) 2 times.
		 */
		Vector2 operator()(std::uniform_random_bit_generator auto& rng) {
			return Vector2{ distributionX(rng), distributionY(rng) };
		}

	private:
		std::uniform_real_distribution<> distributionX;
		std::uniform_real_distribution<> distributionY;
	};


	/**
	 * @brief Represents a uniform distribution inside a circle with a specified radius.
	 */
	class UniformDiskDistribution {
	public:
		UniformDiskDistribution(float radius, Vector2 center = {0.0f, 0.0f}) :
			angleDistribution{ 0, 2.0f * glm::pi<float>() },
			radiusDistribution{ [](float x) {return glm::sqrt(x); } }, radius{ radius }, center{ center } {
		}

		Vector2 operator()(std::uniform_random_bit_generator auto& rng) {
			if (radius == 0) return center;

			float angle = angleDistribution(rng);
			float radius = radiusDistribution(rng);

			//we assume to start with a vector with coordinates (1,0), and rotate it
			Vector2 finalVec{ glm::cos(angle), glm::sin(angle) };
			return center + finalVec * radius;
		}

	private:
		std::uniform_real_distribution<> angleDistribution;
		InvertedCdfDistribution radiusDistribution;
		float radius;
		Vector2 center;
	};
}