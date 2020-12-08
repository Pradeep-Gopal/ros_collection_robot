#include "../include/path_planner.h"

PathPlanner::PathPlanner(){
    grid_size = 0.1;
}

std::vector<geometry_msgs::Point> PathPlanner::AStar(geometry_msgs::Point start, geometry_msgs::Point end) {
    Node start_node;
    start_node.position = start;
    start_node.generate_id();

    Node end_node;
    end_node.position = end;
    end_node.generate_id();

    std::priority_queue<Node, std::vector<Node>, CompareNodeCosts> pq;
    pq.push(start_node);

    std::unordered_map<std::string, Node> visited;

    bool reached_goal = false;

    while (!pq.empty()) {
        Node curr_node = pq.top();
        pq.pop();
        if (!(visited.find(curr_node.id) == visited.end())) // duplicate node in queue
            continue;

        visited[curr_node.id] = curr_node;

        if (curr_node == end_node) { // reached the goal
            ROS_INFO_STREAM("Found the goal");
            reached_goal = true;
            break;
        }

        std::vector<Node> neighbors = checkNeighbors(curr_node,end_node);

        for (Node neighbor:neighbors) {
            if (visited.find(neighbor.id) == visited.end()) {
                pq.push(neighbor);
            } else {

                if (neighbor.g < visited[neighbor.id].g) {
                    visited[neighbor.id].g = neighbor.g;
                    visited[neighbor.id].f = visited[neighbor.id].g + visited[neighbor.id].h;
                    visited[neighbor.id].parent = curr_node.position;
                    pq.push(neighbor);
                }
            }
        }
    }

    // back-track
    std::vector<geometry_msgs::Point> path;

    if (!reached_goal)
        ROS_WARN_STREAM("Unable to find path");
    else {
        Node parent_node = visited[end_node.id];

        while(true) {
            path.push_back(parent_node.position);

            std::stringstream ss;
            ss << std::fixed << std::setprecision(2) << parent_node.parent.x
               << parent_node.parent.y;
            std::string new_parent_id = ss.str();
            new_parent_id.erase(std::remove(new_parent_id.begin(), new_parent_id.end(), '.'), new_parent_id.end());

            parent_node = visited[new_parent_id];

            if (parent_node == start_node)
                break;
        }
    }

    std::reverse(path.begin(),path.end());

    return path;
}

std::vector<Node> PathPlanner::checkNeighbors(Node & curr_node, Node & end_node) {
    std::vector<std::pair<double,double>> directions = {std::pair<double,double>(0,-grid_size),
                                                  std::pair<double,double>(0,grid_size),
                                                  std::pair<double,double>(-grid_size,0),
                                                  std::pair<double,double>(grid_size,0),
                                                  std::pair<double,double>(-grid_size,-grid_size),
                                                  std::pair<double,double>(-grid_size,grid_size),
                                                  std::pair<double,double>(grid_size,-grid_size),
                                                  std::pair<double,double>(grid_size,grid_size) };

    std::vector<Node> neighbors;
    for(std::pair<double,double> dir: directions){
        geometry_msgs::Point new_position;
        double move_cost;

        if(dir.first == 0 || dir.second == 0)
            move_cost = 1;
        else
            move_cost = sqrt(2);

        new_position.x = curr_node.position.x + dir.first;
        new_position.y = curr_node.position.y + dir.second;

        if (!map_.insideObstacle(new_position)) {
            Node child;
            child.position = new_position;
            child.parent = curr_node.position;
            child.g = curr_node.g + move_cost;
            child.h = sqrt(pow(new_position.x-end_node.position.x,2) +
                           pow(new_position.y-end_node.position.y,2));
            child.f = child.g + child.h;
            child.generate_id();
            neighbors.push_back(child);
        }
    }
    return neighbors;
}
