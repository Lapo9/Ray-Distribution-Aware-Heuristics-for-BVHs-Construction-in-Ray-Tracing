#pragma once

#include <vector>

#include "Bvh.h"

namespace pah {
	class TopLevel {
	public:
		TopLevel(const vector<Triangle>& triangles, vector<Bvh>& bvhs);

		void build();

	private:
		const vector<Triangle>& triangles;
		vector<Bvh>& bvhs;
	};

}
