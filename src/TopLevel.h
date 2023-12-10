#pragma once

#include <vector>

#include "Bvh.h"

namespace pah {
	class TopLevel {
	public:
		template<std::same_as<Bvh>... Bvh>
		TopLevel(const std::vector<Triangle>& triangles, Bvh&&... bvhs) : triangles{ triangles } {
			(this->bvhs.emplace_back(std::move(bvhs)), ...);
		}

		virtual void build() = 0;
		virtual void update() = 0;
		virtual const std::vector<Bvh*>& containedIn(const Vector3&) = 0;

		const std::vector<Bvh>& getBvhs() const;
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
		const std::vector<Bvh*>& containedIn(const Vector3&) override;
	};
}
