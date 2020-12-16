/**
 * @file order_manager.cpp
 * @author Pradeep Gopal, Justin Albrecht Govind Ajith Kumar
 * @copyright MIT License
 * @brief Source File for the Line Class
 * This is the Source file for the order_manager class.
 * Takes care of managing orders to be fulfilled by the robot
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

#include "../include/order_manager.h"

OrderManager::OrderManager()
    :map_object_(clearance_) {

    collection_sub_ = nh_.subscribe("/collect_cube", 10,
                                    &OrderManager::collectionCallback, this);
}

/**
 * @brief      Callback to the collection objects
 * @param      Input to the callback
 * @return None
 */
void OrderManager::collectionCallback(
        const ros_collection_robot::Cube::ConstPtr& msg) {
    bool cube_exists = false;
    int idx = 0;
    for (int i = 0; i < cube_locations_.size(); i++) {
        double distance = sqrt(pow(msg->x-cube_locations_[i].x, 2)+
                               pow(msg->y-cube_locations_[i].y, 2));

        if (distance < 1) {
            idx = i;
            cube_exists = true;
            break;
        }
    }

    if (cube_exists && msg->type[0] == cubes_[idx]) {
        deleteCube(cube_names_[idx]);
    } else {
        ROS_WARN_STREAM("Cube not found.");
    }
}

/**
 * @brief      Generates a random order with the cubes spawned inside 
 * the warehouse
 * 
 * @return None
 */
void OrderManager::generateOrder() {
        ROS_INFO_STREAM("Generating Order ...");
        std::vector<char>available_cubes =
                {'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H'};
        srand(time(NULL));  // to generate a truly random number every time

        for (int i = 0; i < total_cubes_; i++) {
            int random_cube_idx;
            random_cube_idx = (rand() % (available_cubes.size()));
            cubes_.push_back(available_cubes[random_cube_idx]);
        }

        std::vector<int>cubes_selected;
        while (order_.size() < order_size_) {
            int order_idx;
            order_idx = (rand() % (cubes_.size()));
            if (std::count(cubes_selected.begin(),
                           cubes_selected.end(), order_idx)) {
            } else {
                cubes_selected.push_back(order_idx);
                order_.push_back(cubes_[order_idx]);
            }
        }

        std::stringstream  ss;
        std::stringstream  order_ss;
        for (int i = 0; i < order_.size(); ++i) {
            if (i == order_.size() - 1)
                ss << order_[i];
            else
                ss << order_[i] << ", ";
            order_ss << order_[i];
        }

        ROS_INFO_STREAM("ORDER : [" << ss.str() << "]");

        nh_.setParam("order", order_ss.str());
}

/**
 * @brief      Spwans the cubes at random locations inside the map
 * 
 * @return None
 */
void OrderManager::spawnCubes() {
    ROS_INFO_STREAM("Spawning cubes");
    srand(time(NULL));

    std::string model_dir = ros::package::getPath
            ("ros_collection_robot") + "/models";

    ros::ServiceClient spawn_client =
            nh_.serviceClient<gazebo_msgs::SpawnModel>
                    ("gazebo/spawn_sdf_model");
    gazebo_msgs::SpawnModel spawn;

    gazebo_msgs::SpawnModel pedestal;

    std::string model_path = model_dir + "/pedestal/model.sdf";
    std::ifstream ifs(model_path);
    std::string xml = "";
    for (std::string line; std::getline(ifs, line);)
        xml += line;
    pedestal.request.model_xml = xml;


    double min_dist = 2;
    std::vector<geometry_msgs::Point> locations;
    int count = 0;
    for (char c : cubes_) {
        geometry_msgs::Point pt;
        while (true) {
            pt.x = static_cast<double>(rand() % max_x_) / 100;
            pt.y = static_cast<double>(rand() % max_y_) / 100;
            if (!map_object_.insideObstacle(pt)) {
                bool too_close = false;
                for (geometry_msgs::Point loc : locations) {
                    double dist = sqrt(pow(pt.x-loc.x, 2) +
                            pow(pt.y-loc.y, 2));
                    if (dist <= min_dist) {
                        too_close = true;
                    }
                }
                if (!too_close) {
                    locations.push_back(pt);
                    break;
                }
            }
        }

        cube_locations_.push_back(pt);

        std::string name = "cube_";
        name.push_back(c);
        name += std::to_string(count);
        spawn.request.model_name = name;
        cube_names_.push_back(name);
        pedestal.request.model_name = "pedestal" + name;

        std::string model_path = model_dir + "/cube_" + c + "/model.sdf";
        std::ifstream ifs(model_path);
        std::string xml = "";
        for (std::string line; std::getline(ifs, line);)
            xml += line;
        spawn.request.model_xml = xml;

        geometry_msgs::Pose pose;
        pose.position.x = pt.x;
        pose.position.y = pt.y;
        pose.position.z = 0;

        pedestal.request.initial_pose = pose;

        pose.position.z = 0.101;
        spawn.request.initial_pose = pose;

        spawn_client.call(pedestal);
        ros::Duration(0.5).sleep();
        spawn_client.call(spawn);
        count++;
    }
}

/**
 * @brief      Deletes the cubes from the world once its collected 
 * by the robot
 *
 * @param[in]  id Id of the cube
 */
void OrderManager::deleteCube(std::string name) {
    ros::ServiceClient delete_client =
            nh_.serviceClient<gazebo_msgs::DeleteModel>
                    ("gazebo/delete_model");
    gazebo_msgs::DeleteModel delete_model;

    if (std::find(delete_cubes_.begin(),
                  delete_cubes_.end(), name) == delete_cubes_.end()) {
        delete_model.request.model_name = name;
        delete_client.call(delete_model);
        delete_cubes_.push_back(name);
    }
}

/**
 * @brief      Gets the total cubes.
 *
 * @return     The total cubes.
 */
int OrderManager::getTotalCubes() {
    return total_cubes_;
}

/**
 * @brief      Gets the order size.
 *
 * @return     The order size.
 */
int OrderManager::getOrderSize() {
    return order_size_;
}

/**
 * @brief      Gets the order.
 *
 * @return     The order.
 */
std::vector<char> OrderManager::getOrder() {
    return order_;
}

/**
 * @brief      Gets the cubes.
 *
 * @return     The cubes.
 */
std::vector<char> OrderManager::getCubes() {
    return cubes_;
}
