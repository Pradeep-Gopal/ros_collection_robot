#include "../include/order_manager.h"

OrderManager::OrderManager(ros::NodeHandle& nh)
    :map_object_(clearance_){
    nh_ = nh;
}

void OrderManager::generateOrder() {
        ROS_INFO_STREAM("Generating Order ...");
        std::vector<char>available_cubes = {'A','B','C','D','E','F','G','H'};
        int random_cube_idx;
        srand(time(NULL));// to generate a truly random number every time

        for(int i = 0; i < total_cubes_; i++){
            random_cube_idx = (rand() % (available_cubes.size()));
            cubes_.push_back(available_cubes[random_cube_idx]);
        }

        int order_idx;
        std::vector<int>cubes_selected;
        while(order_.size()<4){
            order_idx = (rand() % (cubes_.size()));
            if (std::count(cubes_selected.begin(), cubes_selected.end(), order_idx)){
            }
            else{
                cubes_selected.push_back(order_idx);
                order_.push_back(cubes_[order_idx]);
            }
        }

        std::stringstream  ss;
        std::stringstream  order_ss;
        for(int i = 0; i < order_.size(); ++i){
            if (i == order_.size() - 1)
                ss << order_[i];
            else
                ss << order_[i] << ", ";
            order_ss << order_[i];
        }

        ROS_INFO_STREAM("ORDER : [" << ss.str() << "]");

        nh_.setParam("order", order_ss.str());
}

void OrderManager::spawnCubes() {
    ROS_INFO_STREAM("Spawning cubes");
    srand(time(NULL));

    std::string model_dir = ros::package::getPath("ros_collection_robot") + "/models";

    ros::ServiceClient spawn_client = nh_.serviceClient<gazebo_msgs::SpawnModel>("gazebo/spawn_sdf_model");
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
    for (char c:cubes_) {
        geometry_msgs::Point pt;
        while (true) {
            pt.x = (double) (rand() % (1500)) / 100;
            pt.y = (double) (rand() % (1500)) / 100;
            if (!map_object_.insideObstacle(pt)) {
                bool too_close = false;
                for (geometry_msgs::Point loc:locations){
                    double dist = sqrt(pow(pt.x-loc.x,2)+pow(pt.y-loc.y,2));
                    if (dist <= min_dist) {
                        too_close = true;
                    }
                }
                if(!too_close) {
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

void OrderManager::deleteCube(geometry_msgs::Point location, char type){
    ros::ServiceClient delete_client = nh_.serviceClient<gazebo_msgs::DeleteModel>("gazebo/delete_model");
    gazebo_msgs::DeleteModel delete_model;

    bool cube_exists = false;
    int idx;
    for (int i = 0; i < cube_locations_.size(); i++){
        double distance = sqrt(pow(location.x-cube_locations_[i].x,2)+
                               pow(location.y-cube_locations_[i].y,2));

        if (distance < 1) {
            idx = i;
            cube_exists = true;
            break;
        }
    }

    if (cube_exists && type == cubes_[idx]){
        delete_model.request.model_name = cube_names_[idx];
        delete_client.call(delete_model);
    } else
        ROS_WARN_STREAM("Cube not found.");
}

int OrderManager::getTotalCubes(){
    return total_cubes_;
}

int OrderManager::getOrderSize(){
    return order_size_;
}

std::vector<char> OrderManager::getOrder(){
    return order_;
}

std::vector<char> OrderManager::getCubes(){
    return cubes_;
}
