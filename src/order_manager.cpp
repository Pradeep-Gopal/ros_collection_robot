#include "../include/order_manager.h"

void OrderManager::generateOrder(){
    ROS_INFO_STREAM("Generating Order ...");
    std::vector<char>available_cubes = {'A','B','C','D','E','F','G','H'};
    int random_cube_idx;
    srand(time(NULL));// to generate a truly random number every time
    for(int i = 0; i < total_cubes_; i++){
        random_cube_idx = (rand() % (available_cubes.size()));
        cubes_.push_back(available_cubes[random_cube_idx]);
    }
    for(auto cube:cubes_){
        ROS_INFO_STREAM("CUBE : "<<cube);
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
    for(auto order:order_){
        ROS_INFO_STREAM("ORDER : "<<order);
    }
    ROS_INFO_STREAM("Random Cubes generated ...");
}

void OrderManager::spawnCubes() {
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

        std::string name = "cube_";
        name.push_back(c);
        name += std::to_string(count);
        spawn.request.model_name = name;
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

        pose.position.z = 0.081;
        spawn.request.initial_pose = pose;

        ROS_INFO_STREAM(name << " (" << pt.x << ", " << pt.y << ")");
        spawn_client.call(pedestal);
        ros::Duration(0.5).sleep();
        spawn_client.call(spawn);
        count++;
    }
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
