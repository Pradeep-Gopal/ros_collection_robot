#include"../include/node.h"

Node::Node(){
    g = 0;
    h = 0;
    f = 0;
}

//void Node::generate_id() {
//    int precision = 2;
//    std::stringstream ss;
//    ss << std::fixed << std::setprecision(precision) << position.x << position.y;
//    id = ss.str();
//    id.erase(std::remove(id.begin(), id.end(), '.'), id.end());
//}

bool Node::operator==(const Node& node_2)  {
    return ((std::abs(position.x - node_2.position.x) < 0.001) && (std::abs(position.y - node_2.position.y) < 0.001));
}