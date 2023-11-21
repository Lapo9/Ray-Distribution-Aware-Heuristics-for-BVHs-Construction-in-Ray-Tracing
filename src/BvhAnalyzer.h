#pragma once

#include <vector>
#include <string>
#include <functional>

#include "Utilities.h"
#include "Bvh.h"

namespace pah {
	class BvhAnalyzer {
	public:
		using BvhAnalyzerActionType = function<void(const Bvh::Node& current, const Bvh::Node& root, string& nodeLog)>;

		struct GlobalInfo {
			int numberOfNodes;
			int numberOfLeaves;
			int maxLevel;
		};


		string analyze(const Bvh& bvh) const;

		string analyze(const Bvh::Node& node, int currentLevel) const;

	private:
		vector<BvhAnalyzerActionType> actions;
	};
}