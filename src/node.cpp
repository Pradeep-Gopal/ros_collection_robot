#include"../include/node.h"

Node::Node(geometry_msgs::Point point){
    position = point;
    cost_so_far = 0; //g
    heuristic_cost = 0; //h
    total_cost = 0; //f


}
bool Node::operator==(const Node& node_2)  {
    return (position.x == node_2.position.x) && (position.y == node_2.position.y);
}