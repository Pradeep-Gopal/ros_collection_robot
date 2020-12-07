#include "../include/path_planner.h"


std::vector<geometry_msgs::Point> PathPlanner::AStar(geometry_msgs::Point start_point,geometry_msgs::Point end_point){
    Node start_node(start_point);
    Node end_node(end_point);


//    child.cost_so_far = curr_node.cost_so_far + move_cost;
//    child.heuristic_cost = sqrt(pow(new_position.x-end_node.position.x,2)+
//                                pow(new_position.y-end_node.position.y,2));
//    child.total_cost = child.cost_so_far + child.heuristic_cost;


    /**
     *     create pq
     *     put start node in pq
     *     loop through the pq
     *          check if position is in goal
     *              if yes, break out
     *          run check neighbours
     *          for neighbour in neighbours loop through neighbours
     *              check if we have been to the node before
     *                  if so, then replace the
     *
     *
     */

}

std::vector<geometry_msgs::Point> checkNeighbors(Node& curr_node, Node& end_node){
    std::vector<std::pair<int,int>> directions = {std::pair<int,int>(0,-1),
                                                  std::pair<int,int>(0,1),
                                                  std::pair<int,int>(-1,0),
                                                  std::pair<int,int>(1,0),
                                                  std::pair<int,int>(-1,-1),
                                                  std::pair<int,int>(-1,1),
                                                  std::pair<int,int>(1,-1),
                                                  std::pair<int,int>(1,1)};

    std::vector<Node> neighbors;
    for(std::pair<int,int> dir: directions){
        geometry_msgs::Point new_position;
        double move_cost;

        if(dir.first == 1 && dir.second == 1)
            move_cost = 1.414;
        else
            move_cost = 1;

        new_position.x = curr_node.position.x + dir.first;
        new_position.y = curr_node.position.y + dir.second;
        if(!map_.insideObstacle(new_position)){
            Node child(new_position);
            child.parent = curr_node.position;
            neighbors.push_back(child);
        }
    }
    return neighbors;
}

std::vector<geometry_msgs::Point> PathPlanner::EulerPath(){

}