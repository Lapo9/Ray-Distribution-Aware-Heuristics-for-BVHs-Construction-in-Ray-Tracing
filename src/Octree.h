#pragma once

#include <array>
#include <functional>
#include <memory>

#include "Utilities.h"

namespace pah {
	//TODO this class is just a draft
	template <typename T>
	class Octree {
	public:

		struct Node {
			T data;
			std::array<std::unique_ptr<Node>, 8> children;

			bool isLeaf() const {
				for (auto& child : children) {
					if (child != nullptr) { return false; }
				}
				return true;
			}
		};

		Octree(std::function<void(const T& rootData)> buildOctree) : buildOctree{ buildOctree } {}

		void build(const T& rootData) {
			buildOctree(rootData);
		}

		T& get(Vector3 point) const {
			return root.data; //TODO just a placeholder to compile other code
		}

		Node& getRoot() const {
			return root;
		}

	private:
		Node root;
		std::function<void(const T& rootData)> buildOctree;
	};
}
