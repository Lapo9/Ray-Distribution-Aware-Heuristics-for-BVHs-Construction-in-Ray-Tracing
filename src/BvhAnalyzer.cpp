#include "BvhAnalyzer.h"
#include "CustomJson.h"
#include "../libs/json.hpp"

using namespace std;
using json = nlohmann::json;

json pah::BvhAnalyzer::analyze(const Bvh& bvh) {
    log = json{}; //initialize log string
    globalInfo = GlobalInfo{};

    analyzeNode(bvh.getRoot(), bvh, 0);

    //log["globalInfo"] = globalInfo;
    return log;
}

void pah::BvhAnalyzer::analyzeNode(const Bvh::Node& node, const Bvh& bvh, int currentLevel) {
    json localLog;
    coreAction(node, currentLevel, localLog);
    for (const auto& action : actions) {
        action(node, bvh, localLog);
    }
    
    log.push_back(localLog); //add log of this node to the global log

    //recursion
    if (node.leftChild != nullptr) analyzeNode(*node.leftChild, bvh, currentLevel);
    if (node.rightChild != nullptr) analyzeNode(*node.rightChild, bvh, currentLevel);
}

void pah::BvhAnalyzer::coreAction(const Bvh::Node& node, int currentLevel, json& localLog) {
    globalInfo.maxLevel = currentLevel > globalInfo.maxLevel ? currentLevel : globalInfo.maxLevel;
    globalInfo.numberOfNodes++;
    globalInfo.numberOfLeaves += node.isLeaf();

    localLog["core"] = node;
}
