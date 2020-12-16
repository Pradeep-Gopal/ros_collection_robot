/**
 * @file path_planner.cpp
 * @author Pradeep Gopal, Justin Albrecht Govind Ajith Kumar
 * @copyright MIT License
 * @brief Source file for the Path planner Class
 * This is the Source file for the path_planner class.
 * Path planning using A star algorithm
 */

/**
 *MIT License
 *Copyright (c) 2020 Pradeep Gopal, Justin Albrecht, Govind Ajith Kumar
 *Permission is hereby granted, free of charge, to any person obtaining a copy
 *of this software and associated documentation files (the "Software"), to deal
 *in the Software without restriction, including without limitation the rights
 *to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *copies of the Software, and to permit persons to whom the Software is
 *furnished to do so, subject to the following conditions:
 *The above copyright notice and this permission notice shall be included in all
 *copies or substantial portions of the Software.
 *THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *SOFTWARE.
 */

#include "../include/path_planner.h"

/**
 * @brief      User defined path planner constructor
 */
PathPlanner::PathPlanner()
    :map(clearance_) {
        grid_size = 0.1;
        height = 15;
        width = 15;
}

/**
 * @brief      A-star path planner for the robot
 *
 * @param[in]  start  The start node
 * @param[in]  end    The end node
 *
 * @return     A vector of geometry_msgs::Point that the
 * robot should traverse as per the optimal solution
 */
std::vector<geometry_msgs::Point>
        PathPlanner::AStar(geometry_msgs::Point start,
                           geometry_msgs::Point end) {
    std::stringstream ss1, ss2, ss3, ss4;
    ss1 << std::fixed << std::setprecision(1) << start.x;
    start.x = std::stod(ss1.str());

    ss2 << std::fixed << std::setprecision(1) << start.y;
    start.y = std::stod(ss2.str());

    ss3 << std::fixed << std::setprecision(1) << end.x;
    end.x = std::stod(ss3.str());

    ss4 << std::fixed << std::setprecision(1) << end.y;
    end.y = std::stod(ss4.str());

    Node start_node;
    start_node.position = start;
    start_node.id = generate_node_id(start);

    Node end_node;
    end_node.position = end;
    end_node.id = generate_node_id(end);

    std::priority_queue<Node, std::vector<Node>, CompareNodeCosts> pq;
    pq.push(start_node);

    std::unordered_map<std::string, Node> visited;

    bool reached_goal = false;

    while (!pq.empty()) {
        Node curr_node = pq.top();
        pq.pop();
        if (!(visited.find(curr_node.id)
        == visited.end()))  // duplicate node in queue
            continue;

        visited[curr_node.id] = curr_node;

        if (curr_node == end_node) {  // reached the goal
            reached_goal = true;
            break;
        }

        std::vector<Node> neighbors = checkNeighbors(curr_node, end_node);

        for (Node neighbor : neighbors) {
            if (visited.find(neighbor.id) == visited.end()) {
                pq.push(neighbor);
            } else {
                if (neighbor.g < visited[neighbor.id].g) {
                    visited[neighbor.id].g = neighbor.g;
                    visited[neighbor.id].f
                     = visited[neighbor.id].g + visited[neighbor.id].h;
                    visited[neighbor.id].parent = curr_node.position;
                    pq.push(neighbor);
                }
            }
        }
    }

    std::vector<geometry_msgs::Point> path;

    if (!reached_goal) {
        ROS_WARN_STREAM("Unable to find path");
    } else {
        Node parent_node = visited[end_node.id];

        while (true) {
            path.push_back(parent_node.position);
            std::string new_parent_id = generate_node_id(parent_node.parent);
            parent_node = visited[new_parent_id];

            if (parent_node == start_node)
                break;
        }
    }

    std::reverse(path.begin(), path.end());

    return path;
}

/**
 * @brief      Returns list of neighbors for a particular node
 *  in the geometry_msgs::Point format
 *
 * @param      curr_node  The curr node
 * @param      end_node   The end node
 *
 * @return     Returns a vector of geometry_msgs::Point that is 
 * neigbouring to each node in the map where the robot is present
 */
std::vector<Node> PathPlanner::checkNeighbors(Node & curr_node,
 Node & end_node) {
    std::vector<std::pair<double, double>> directions
     = {std::pair<double, double>(0, -grid_size),
      std::pair<double, double>(0, grid_size),
      std::pair<double, double>(-grid_size, 0),
      std::pair<double, double>(grid_size, 0),
      std::pair<double, double>(-grid_size, -grid_size),
      std::pair<double, double>(-grid_size, grid_size),
      std::pair<double, double>(grid_size, -grid_size),
      std::pair<double, double>(grid_size, grid_size) };

    std::vector<Node> neighbors;
    for (std::pair<double, double> dir : directions) {
        geometry_msgs::Point new_position;
        double move_cost;

        if (dir.first == 0 || dir.second == 0)
            move_cost = 1;
        else
            move_cost = sqrt(2);

        new_position.x = curr_node.position.x + dir.first;
        new_position.y = curr_node.position.y + dir.second;

        if (!map.insideObstacle(new_position)) {
            Node child;
            child.position = new_position;
            child.parent = curr_node.position;
            child.g = curr_node.g + move_cost;
            child.h = sqrt(pow(new_position.x-end_node.position.x, 2) +
                           pow(new_position.y-end_node.position.y, 2));
            child.f = child.g + child.h;
            child.id = generate_node_id(new_position);
            neighbors.push_back(child);
        }
    }
    return neighbors;
}

/**
 * @brief      Generates an unique string node ID for 
 * every single node in the map
 *
 * @param[in]  position  The position
 *
 * @return     The node ID
 */
std::string PathPlanner::generate_node_id(geometry_msgs::Point position) {
    int precision = 1;
    std::stringstream ss;
    ss << std::fixed << std::setprecision(precision)
     << position.x << position.y;
    std::string id = ss.str();
    id.erase(std::remove(id.begin(), id.end(), '.'), id.end());

    return id;
}
