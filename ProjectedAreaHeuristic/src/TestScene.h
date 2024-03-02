#pragma once

#include <vector>
#include <ranges>
#include "Utilities.h"
#include "Bvh.h"
#include "TopLevel.h"
#include "RayCaster.h"

namespace pah {
	class TestScene {
	public:
		TestScene(std::vector<Triangle>&& triangles, std::vector<RayCaster<>*>&& rayCasters, TopLevel&& topLevel, Bvh&& sahBvh) :
			triangles{ std::move(triangles) }, rayCasters{ std::move(rayCasters) }, topLevel{ &topLevel }, sahBvh{ std::move(sahBvh) } {
		}

		void build() {
			topLevel->build(triangles);

			auto trianglesSah = triangles;
			auto trianglesSahPtrs = trianglesSah | std::views::transform([](const Triangle& t) {return &t; }) | std::ranges::to<std::vector>();
			sahBvh.build(trianglesSahPtrs);
		}

		void traverse() {
			// TODO
		}

	private:
		std::vector<Triangle> triangles;
		std::vector<RayCaster<>*> rayCasters;
		TopLevel* topLevel;
		Bvh sahBvh;
	};
}
