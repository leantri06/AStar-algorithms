#include "PathNode.h"

PathNode::PathNode() {
    name = "";
    f = 0.0;
    g = 0.0;
    h = 0.0;
    next = nullptr;
}