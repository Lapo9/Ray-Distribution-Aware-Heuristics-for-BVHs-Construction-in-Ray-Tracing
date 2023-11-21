#include "BvhAnalyzer.h"

using namespace std;

string pah::BvhAnalyzer::analyze(const Bvh& bvh) const {
    analyze(bvh.getRoot(), 0);
}

string pah::BvhAnalyzer::analyze(const Bvh::Node& node, int currentLevel) const {
    
}
