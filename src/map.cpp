#include<cstdlib>
#include <ctime>
#include "ros/ros.h"
#include "../include/map.h"

Map::Map(){
    std::string fname = ros::package::getPath("ros_collection_robot") + "/config/map.yaml";
    clearance_ = 0.2;
    parseYAML(fname);

    for (Polygon poly:obstacles_){
        offset_obstacles_.push_back(offsetPolygon(poly));
    }

}

bool Map::insideObstacle(geometry_msgs::Point check_point){
    for (Polygon poly:offset_obstacles_){
        if (poly.insideObject(check_point))
            return true;
    }
    return false;
}

void Map::parseYAML(std::string fname) {
    YAML::Node map = YAML::LoadFile(fname);
    for (auto it = map.begin(); it != map.end(); ++it){
        YAML::Node key = it->first;
        YAML::Node obs = map[key.as<std::string>()];
        if (obs["type"].as<std::string>() == "rectangle"){
            std::vector<double> start = obs["start"].as<std::vector<double>>();
            double width = obs["width"].as<double>();
            double height = obs["height"].as<double>();
            double theta = obs["theta"].as<double>();

            obstacles_.push_back(polyFromRect(start[0],start[1],width,height,theta));
        }
        else if (obs["type"].as<std::string>() == "circle"){
            std::vector<double> center = obs["center"].as<std::vector<double>>();
            double radius = obs["radius"].as<double>();
            obstacles_.push_back(polyFromCircle(center[0],center[1],radius));
        }
        else if (obs["type"].as<std::string>() == "polygon") {
            YAML::Node coords = obs["vertices"];
            std::vector <geometry_msgs::Point> vertices;
            for(int i = 0; i < coords.size(); ++i){
                geometry_msgs::Point vertex;
                vertex.x = coords[i][0].as<double>();
                vertex.y = coords[i][1].as<double>();
                vertices.push_back(vertex);
            }
            obstacles_.push_back(Polygon(vertices));
        }
    }
}


Polygon Map::polyFromRect(double xs, double ys, double width, double height, double theta){
    std::vector <geometry_msgs::Point> vertices;

    const double pi = 3.14159;
    double theta_rad = theta * (pi/180);
    double l = sqrt(pow(height,2)+pow(width,2));
    double alpha = atan2(height,width);

    geometry_msgs::Point point;
    point.x = xs;
    point.y = ys;
    vertices.push_back(point);

    point.x = xs+(width*cos(theta_rad));
    point.y = ys+(width*sin(theta_rad));
    vertices.push_back(point);

    point.x = xs+(l*cos(theta_rad+alpha));
    point.y = ys+(l*sin(theta_rad+alpha));
    vertices.push_back(point);

    point.x = xs+(height*cos(theta_rad+(pi/2)));
    point.y = ys+(height*sin(theta_rad+(pi/2)));
    vertices.push_back(point);

    return Polygon(vertices);
}

Polygon Map::polyFromCircle(double xc, double yc, double radius){
    std::vector <geometry_msgs::Point> vertices;
    geometry_msgs::Point point;

    const double pi = 3.14159;

    int sides = 24;
    double theta = 0;
    for (int i = 0; i<sides; i++) {
        point.x = xc + (radius * sin(theta));
        point.y = yc + (radius * cos(theta));
        theta += (2*pi)/sides;
        vertices.push_back(point);
    }

    return Polygon(vertices);
}

Polygon Map::offsetPolygon(Polygon poly){
    int scale_factor = 1000000;

    ClipperLib::Path subj;
    ClipperLib::Paths solution;
    ClipperLib::ClipperOffset co;
    std::vector<geometry_msgs::Point> vertices = poly.getVertices();
    for (geometry_msgs::Point pt:vertices){
        subj << ClipperLib::IntPoint(pt.x*scale_factor,pt.y*scale_factor);
    }

    co.AddPath(subj,ClipperLib::jtMiter,ClipperLib::etClosedPolygon);
    co.Execute(solution,clearance_*scale_factor);

    std::vector<geometry_msgs::Point> offset_vertices;
    geometry_msgs::Point offset_point;
    for (ClipperLib::IntPoint int_point:solution[0]){
        offset_point.x = (double)int_point.X/scale_factor;
        offset_point.y = (double)int_point.Y/scale_factor;
        offset_vertices.push_back(offset_point);
    }
    return Polygon(offset_vertices);
}

void Map::addObstacle(Polygon new_poly) {
    obstacles_.push_back(new_poly);
    Polygon offset_new_poly = offsetPolygon(new_poly);
    offset_obstacles_.push_back(offset_new_poly);
}

