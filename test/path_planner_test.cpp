#include <gtest/gtest.h>
#include "../include/path_planner.h"

TEST(pathPlanner, pathPlannerTest) {
    Map map(0.1);
    PathPlanner planner;
    EXPECT_DOUBLE_EQ(planner.grid_size, 0.1);
    EXPECT_DOUBLE_EQ(planner.height, 15);
    EXPECT_DOUBLE_EQ(planner.width, 15);
}

TEST(AStar, AStarTest) {
    PathPlanner planner;
    geometry_msgs::Point start, end, waypoint1, waypoint2, waypoint3, waypoint4,
            waypoint5;
    start.x = 1;
    start.y = 1;
    start.z = 0;

    end.x = 1;
    end.y = 1.5;
    end.z = 0;

    waypoint1.x = 1;
    waypoint1.y = 1.1;
    waypoint1.z = 0;

    waypoint2.x = 1;
    waypoint2.y = 1.2;
    waypoint2.z = 0;

    waypoint3.x = 1;
    waypoint3.y = 1.3;
    waypoint3.z = 0;

    waypoint4.x = 1;
    waypoint4.y = 1.4;
    waypoint4.z = 0;

    waypoint5.x = 1;
    waypoint5.y = 1.5;
    waypoint5.z = 0;

    std::vector<geometry_msgs::Point> Path = planner.AStar(start, end);
    std::vector<geometry_msgs::Point> path_check;
    path_check.push_back(waypoint1);
    path_check.push_back(waypoint2);
    path_check.push_back(waypoint3);
    path_check.push_back(waypoint4);
    path_check.push_back(waypoint5);

for(int i = 0; i < Path.size(); i++) {
        EXPECT_NEAR(Path[i].x, path_check[i].x, 0.001);
        EXPECT_NEAR(Path[i].y, path_check[i].y, 0.001);
        EXPECT_NEAR(Path[i].z, path_check[i].z, 0.001);
    }
}

TEST(checkNeighbours, testCheckNeighbours) {
    PathPlanner planner;
    Node curr_node;
    geometry_msgs::Point position;
    position.x = 1;
    position.y = 1;
    position.z = 0;
    geometry_msgs::Point parent;
    parent.x = 0;

    parent.y = 0;
    parent.z = 0;
    double g = 0;
    double h = 1;
    double f = 1;
    std::string id = "23";

    curr_node.position = position;
    curr_node.parent = parent;
    curr_node.f = f;
    curr_node.g = g;
    curr_node.h = h;
    curr_node.id = id;

    Node end_node;

    end_node.position = position;
    end_node.parent = parent;
    end_node.f = f;
    end_node.g = g;
    end_node.h = h;
    end_node.id = id;

    std::vector<Node> neighbours = planner.checkNeighbors(curr_node, end_node);
    EXPECT_EQ(neighbours.size(), 8);
}

TEST(generate_node_id, test_generate_node_id) {
    PathPlanner planner;
    geometry_msgs::Point position;
    position.x = 1;
    position.y = 1;
    position.z = 0;
    EXPECT_EQ(planner.generate_node_id(position), "1010");
}
