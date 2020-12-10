#include"../include/node.h"

Node::Node(){
    g = 0;
    h = 0;
    f = 0;
}

bool Node::operator==(const Node& node_2)  {
    return ((std::abs(position.x - node_2.position.x) < 0.001) && (std::abs(position.y - node_2.position.y) < 0.001));
}