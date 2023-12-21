#pragma once

#include <vector>
#include <optional>

#include "Bvh.h"

namespace pah {
	/**
	 * @brief Base class for the top level structure of the BVH. 
	 * This structure subdivides the BVH into 3d space regions, each of which will have its own BVH.
	 * Generally a region can be of 2 types: with parallel rays (orthographic), and with rays arranged in a conic fashion (perspective).
	 */
	class TopLevel {
	public:
		template<std::same_as<Bvh>... Bvh>
		TopLevel(const std::vector<Triangle>& triangles, Bvh&&... bvhs) : triangles{ triangles } {
			(this->bvhs.emplace_back(std::move(bvhs)), ...);
		}

		/**
		 * @brief Insert the triangles in the specific area they belong to, then builds the BVHs.
		 */
		virtual void build() = 0;

		/**
		 * @brief Updates the region where each triangle belongs to.
		 * TODO implement it in concrete classes, it probably needs to know which parts to reconstruct
		 */
		virtual void update() = 0;

		/**
		 * @brief Given a point, returns the /p Region s it belongs to.
		 */
		virtual std::vector<Bvh*> containedIn(const Vector3&) = 0;

		/**
		 * @brief Returns the /p Bvh s that are part of this /p TopLevel structure.
		 */
		const std::vector<Bvh>& getBvhs() const;

		/**
		 * @brief Returns the /p Triangle s that are part of this /p TopLevel structure.
		 */
		const std::vector<Triangle>& getTriangles() const;

	protected:
		const std::vector<Triangle>& triangles;
		std::vector<Bvh> bvhs;
	};


	class TopLevelAabbs : public TopLevel {
	public:
		template<std::same_as<Bvh>... Bvh>
		TopLevelAabbs(const std::vector<Triangle>& triangles, Bvh&&... bvhs) : TopLevel{ triangles, std::move(bvhs)... } {}

		void build() override;
		void update() override;
		std::vector<Bvh*> containedIn(const Vector3&) override;
	};

	class TopLevelOctree : public TopLevel {
	public:
		//related classes
		struct Node {
			std::vector<Bvh*> bvhs;
			std::array<std::unique_ptr<Node>, 8> children;

			bool isLeaf() const {
				if (leaf.has_value()) return leaf.value(); //if the user set that this node is a leaf, we can shortcut

				for (auto& child : children) {
					if (child != nullptr) { return false; }
				}
				return true;
			}
			void setLeaf(bool value) {
				leaf = value;
			}
			void resetLeaf() {
				leaf.reset();
			}

		private:
			std::optional<bool> leaf;
		};


		template<std::same_as<Bvh>... Bvh>
		TopLevelOctree(const std::vector<Triangle>& triangles, Bvh&&... bvhs) : TopLevel{ triangles, std::move(bvhs)... } {
			//TODO from here
		}

		void build() override;
		void update() override;
		std::vector<Bvh*> containedIn(const Vector3&) override;

	private:
		std::unique_ptr<Node> root;
	};
}
