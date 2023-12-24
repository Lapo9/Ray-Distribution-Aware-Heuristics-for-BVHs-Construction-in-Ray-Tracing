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
		virtual void build();

		/**
		 * @brief Updates the region where each triangle belongs to.
		 * TODO implement it in concrete classes, it probably needs to know which parts to reconstruct
		 */
		virtual void update() = 0;

		/**
		 * @brief Given a point, returns the @p Region s it belongs to.
		 */
		virtual std::vector<Bvh*> containedIn(const Vector3&) = 0;

		/**
		 * @brief Returns the @p Bvh s that are part of this @p TopLevel structure.
		 */
		const std::vector<Bvh>& getBvhs() const;

		/**
		 * @brief Returns the @p Triangle s that are part of this @p TopLevel structure.
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
			Aabb aabb;
			std::vector<Bvh*> bvhs;
			std::array<std::unique_ptr<Node>, 8> children;

			Node() {}
			Node(const Aabb& aabb) : aabb{ aabb } {}

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

		struct Properties {
			int maxLevel;
			/**
			 * @brief If true, the leaves will only contain the regions that fully contain them. 
			 * The issue is that, if @p maxLevel is too low this may create not fully connected regions.
			 */
			bool conservativeApproach;
		};


		template<std::same_as<Bvh>... Bvh>
		TopLevelOctree(const Properties& properties, const std::vector<Triangle>& triangles, Bvh&&... bvhs) : TopLevel{ triangles, std::move(bvhs)... }, properties{ properties }, root{ std::make_unique<Node>() } {
			Aabb sceneAabb = Aabb::minAabb();
			for (const auto& bvh : this->bvhs) {
				sceneAabb += bvh.getInfluenceArea().getBvhRegion().enclosingAabb();
			}

			root->aabb = sceneAabb;
		}

		void build() override;
		void update() override;
		std::vector<Bvh*> containedIn(const Vector3&) override;

		Node& getRoot() const;

	private:
		/**
		 * @brief Recursively creates the nodes of the octree.
		 */
		void buildOctreeRecursive(Node& node, const std::vector<Bvh*>& fatherCollidingRegions, const std::vector<Bvh*>& fatherFullyContainedRegions, int currentLevel = 0);

		/**
		 * @brief Given the relative position of a point to the center of the @p Aabb, returns the index of the @p Node.
		 * For example, if the point is <3,7,4> and the center is <2,8,9>, the relative position is <true, false, false>.
		 * Look at @p TopLevelOctree::indexToPosition to understand the order of the octants.
		 */
		static int positionToIndex(bool x, bool y, bool z);
		/**
		 * @brief Look at \p TopLevelOctree::positionToIndex(bool,bool,bool).
		 */
		static int positionToIndex(const Vector3& pos);
		/**
		 * @brief Octants are logically disposed in this order:
		 *   _________________
		 *  |\    2   \   6   \
		 *  | \--------\-------\
		 *  |\|\________\_______\             
		 *  |0| |   3   |   7   |   4 is the hidden one
		 *   \|\|_______|_______|
		 *    \ |   1   |   5   |
		 *     \|_______|_______|        
		 */
		static Vector3 indexToPosition(int i);

		std::unique_ptr<Node> root;
		Properties properties;
	};
}
