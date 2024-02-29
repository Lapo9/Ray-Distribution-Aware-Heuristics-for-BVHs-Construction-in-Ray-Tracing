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
		//related classes
		struct TraversalResults {
			int bvhsTraversed; /**< How many @p Bvh s we traversed before fnding the hit. */
			int totalBvhs; /**< How many potential @p Bvh s we could have traversed. */
			int intersectionTestsTotal;
			int intersectionTestsWithNodes;
			int intersectionTestsWithTriangles;
			float traversalCostTotal;
			std::unordered_map<const Bvh*, std::pair<float, int>> traversalCostForBvh; //for each BVH holds its cost and how many rays intersected it.
			int intersectionTestsWhenHit;
			int intersectionTestsWithNodesWhenHit;
			int intersectionTestsWithTrianglesWhenHit;
			float costWhenHit;
			const Triangle* closestHit;
			float closestHitDistance;

			bool fallbackBvhSearch;
			int intersectionTestsTotalNonFallback;
			int intersectionTestsWithNodesNonFallback;
			int intersectionTestsWithTrianglesNonFallback;
			int intersectionTestsWhenHitNonFallback;
			int intersectionTestsWithNodesWhenHitNonFallback;
			int intersectionTestsWithTrianglesWhenHitNonFallback; 
			TIME(DurationMs traversalTime;);
			TIME(DurationMs bvhOnlyTraversalTime;);

			bool hit() const {
				return closestHit != nullptr;
			}

			TraversalResults& operator+=(const Bvh::TraversalResults& rhs) {
				bvhsTraversed++;
				intersectionTestsTotal += rhs.intersectionTestsTotal;
				intersectionTestsWithNodes += rhs.intersectionTestsWithNodes;
				intersectionTestsWithTriangles += rhs.intersectionTestsWithTriangles;
				traversalCostTotal += rhs.traversalCost;
				traversalCostForBvh[rhs.bvh].first += rhs.traversalCost;
				traversalCostForBvh[rhs.bvh].second++;
				TIME(bvhOnlyTraversalTime += rhs.traversalTime);

				if (!fallbackBvhSearch) {
					intersectionTestsTotalNonFallback += rhs.intersectionTestsTotal;
					intersectionTestsWithNodesNonFallback += rhs.intersectionTestsWithNodes;
					intersectionTestsWithTrianglesNonFallback += rhs.intersectionTestsWithTriangles;
				}
				
				if (rhs.hit()) {
					intersectionTestsWhenHit += rhs.intersectionTestsTotal;
					intersectionTestsWithNodesWhenHit += rhs.intersectionTestsWithNodes;
					intersectionTestsWithTrianglesWhenHit += rhs.intersectionTestsWithTriangles;
					costWhenHit += rhs.traversalCost;
					if (!fallbackBvhSearch) {
						intersectionTestsWhenHitNonFallback += rhs.intersectionTestsTotal;
						intersectionTestsWithNodesWhenHitNonFallback += rhs.intersectionTestsWithNodes;
						intersectionTestsWithTrianglesWhenHitNonFallback += rhs.intersectionTestsWithTriangles;
					}
				}
				return *this;
			}
		};

		template<std::same_as<Bvh>... Bvhs>
		TopLevel(const std::vector<Triangle>& triangles, Bvh&& fallbackBvh, Bvhs&&... bvhs) : triangles{ triangles }, fallbackBvh{ std::move(fallbackBvh) } {
			(this->bvhs.emplace_back(std::move(bvhs)), ...);
		}

		/**
		 * @brief Insert the triangles in the specific area they belong to, then builds the BVHs.
		 */
		virtual void build(float splitPlaneQualityThreshold, float maxChildrenFatherHitProbabilityRatio);

		/**
		 * @brief Updates the region where each triangle belongs to.
		 * TODO implement it in concrete classes, it probably needs to know which parts to reconstruct
		 */
		virtual void update() = 0;

		/**
		 * @brief Given a point, returns the @p Region s it belongs to.
		 */
		virtual std::vector<const Bvh*> containedIn(const Vector3&) const = 0;

		virtual TraversalResults traverse(const Ray& ray) const;

		/**
		 * @brief Returns the @p Bvh s that are part of this @p TopLevel structure.
		 */
		const std::vector<Bvh>& getBvhs() const;

		/**
		 * @brief Returns the fallback @p Bvh.
		 */
		const Bvh& getFallbackBvh() const;

		/**
		 * @brief Returns the @p Triangle s that are part of this @p TopLevel structure.
		 */
		const std::vector<Triangle>& getTriangles() const;

	protected:
		const std::vector<Triangle>& triangles;
		std::vector<Bvh> bvhs;
		Bvh fallbackBvh; //if none of the other BVHs is hit, this one is used; it will contain every triangle in the scene
	};


	class TopLevelAabbs : public TopLevel {
	public:
		template<std::same_as<Bvh>... Bvhs>
		TopLevelAabbs(const std::vector<Triangle>& triangles, Bvh&& fallbackBvh, Bvhs&&... bvhs) : TopLevel{ triangles, std::move(fallbackBvh), std::move(bvhs)... } {}

		void build(float splitPlaneQualityThreshold, float maxChildrenFatherHitProbabilityRatio) override;
		void update() override;
		std::vector<const Bvh*> containedIn(const Vector3&) const override;
	};


	class TopLevelOctree : public TopLevel {
	public:
		//related classes
		struct NodeTimingInfo {
			DurationMs total;

			void logTotal(DurationMs duration) {
				total = duration;
			}

			/**
			 * @brief Adds together 2 NodeTimingInfo objects.
			 */
			NodeTimingInfo& operator+=(const NodeTimingInfo& lhs) {
				total = total + lhs.total;
				return *this;
			}

			friend NodeTimingInfo& operator+(NodeTimingInfo lhs, const NodeTimingInfo& rhs) {
				return lhs += rhs;
			}

			friend bool operator==(const NodeTimingInfo& lhs, const NodeTimingInfo& rhs) {
				return	lhs.total == rhs.total;
			}
		};

		struct Node {
			Aabb aabb;
			std::vector<Bvh*> bvhs;
			std::array<std::unique_ptr<Node>, 8> children;
			TIME(NodeTimingInfo timingInfo;)

			Node() {
				TIME(timingInfo = NodeTimingInfo{};);
			}
			Node(const Aabb& aabb) : aabb{ aabb } {
				TIME(timingInfo = NodeTimingInfo{};);
			}

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


		template<std::same_as<Bvh>... Bvhs>
		TopLevelOctree(const Properties& properties, const std::vector<Triangle>& triangles, Bvh&& fallbackBvh, Bvhs&&... bvhs) 
			: TopLevel{ triangles, std::move(fallbackBvh), std::move(bvhs)... }, properties{ properties }, root{ std::make_unique<Node>() } {
			Aabb sceneAabb = Aabb::minAabb();

			//the Aabb of the root, must contain all the influence areas in the scene
			for (const auto& bvh : this->bvhs) {
				sceneAabb += bvh.getInfluenceArea()->getBvhRegion().enclosingAabb();
			}

			root->aabb = sceneAabb;
		}

		void build(float splitPlaneQualityThreshold, float maxChildrenFatherHitProbabilityRatio) override;
		void update() override;
		std::vector<const Bvh*> containedIn(const Vector3&) const override;

		Node& getRoot() const;
		INFO(const DurationMs getTotalBuildTime() const;); /**< @brief Returns the time it took to build this @p TopLevelOctree. */
		Properties getProperties() const; /**< @brief Returns the properties of this @p TopLevelOctree. */

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
		INFO(DurationMs totalBuildTime;);
	};
}
