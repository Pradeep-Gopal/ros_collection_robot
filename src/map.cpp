/**
 * @file map.cpp
 * @author Pradeep Gopal, Justin Albrecht Govind Ajith Kumar
 * @copyright MIT License
 * @brief Source file for the map Class
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

#include<cstdlib>
#include <ctime>
#include "ros/ros.h"
#include "../include/map.h"

/**
 * @brief      Constructs a new instance.
 * @param[in]  clearance
 */
Map::Map(double clearance) {
    std::string fname = ros::package::getPath
            ("ros_collection_robot") + "/config/map.yaml";
    clearance_ = clearance;
    parseYAML(fname);

    for (Polygon poly : obstacles_) {
        offset_obstacles_.push_back(offsetPolygon(poly));
    }
}

/**
 * @brief      Checks if a point is inside the obstacle or not
 *
 * @param[in]  Point  Geometry_msgs::Point
 *
 * @return     True if point is present inside obstacle space
 */
bool Map::insideObstacle(geometry_msgs::Point check_point) {
    for (Polygon poly : offset_obstacles_) {
        if (poly.insideObject(check_point))
            return true;
    }
    return false;
}

/**
 * @brief      Reads the YAML file
 * @param[in]  fname path to the YAML file
 */
void Map::parseYAML(std::string fname) {
    YAML::Node map = YAML::LoadFile(fname);
    for (auto it = map.begin(); it != map.end(); ++it) {
        YAML::Node key = it->first;
        YAML::Node obs = map[key.as<std::string>()];
        if (obs["type"].as<std::string>() == "rectangle") {
            std::vector<double> start = obs["start"].as<std::vector<double>>();
            double width = obs["width"].as<double>();
            double height = obs["height"].as<double>();
            double theta = obs["theta"].as<double>();

            obstacles_.push_back(polyFromRect(start[0],
                                              start[1], width, height, theta));
        } else if (obs["type"].as<std::string>() == "circle") {
            std::vector<double> center =
                    obs["center"].as<std::vector<double>>();
            double radius = obs["radius"].as<double>();
            obstacles_.push_back(polyFromCircle(center[0], center[1], radius));
        } else if (obs["type"].as<std::string>() == "polygon") {
            YAML::Node coords = obs["vertices"];
            std::vector <geometry_msgs::Point> vertices;
            for (int i = 0; i < coords.size(); ++i) {
                geometry_msgs::Point vertex;
                vertex.x = coords[i][0].as<double>();
                vertex.y = coords[i][1].as<double>();
                vertices.push_back(vertex);
            }
            obstacles_.push_back(Polygon(vertices));
        }
    }
}

/**
 * @brief      Creates the rectangle shaped obstacles from the Warehouse
 * @return     Polygon
 */
Polygon Map::polyFromRect(double xs, double ys, double width,
                          double height, double theta) {
    std::vector <geometry_msgs::Point> vertices;

    const double pi = 3.14159;
    double theta_rad = theta * (pi/180);
    double l = sqrt(pow(height, 2)+pow(width, 2));
    double alpha = atan2(height, width);

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

/**
 * @brief      Creates the Circular shaped obstacles from the Warehouse
 * @return     Polygon
 */
Polygon Map::polyFromCircle(double xc, double yc, double radius) {
    std::vector <geometry_msgs::Point> vertices;
    geometry_msgs::Point point;

    const double pi = 3.14159;

    int sides = 24;
    double theta = 0;
    for (int i = 0; i < sides; i++) {
        point.x = xc + (radius * sin(theta));
        point.y = yc + (radius * cos(theta));
        theta += (2*pi)/sides;
        vertices.push_back(point);
    }

    return Polygon(vertices);
}

/**
 * @brief     Offsets the polygon by some distance
 *
 * @param[in]  Polygon
 *
 * @return     Offsetted polygon
 */
Polygon Map::offsetPolygon(Polygon poly) {
    int scale_factor = 1000000;

    ClipperLib::Path subj;
    ClipperLib::Paths solution;
    ClipperLib::ClipperOffset co;
    std::vector<geometry_msgs::Point> vertices = poly.getVertices();
    for (geometry_msgs::Point pt : vertices) {
        subj << ClipperLib::IntPoint(pt.x*scale_factor, pt.y*scale_factor);
    }

    co.AddPath(subj, ClipperLib::jtMiter, ClipperLib::etClosedPolygon);
    co.Execute(solution, clearance_*scale_factor);

    std::vector<geometry_msgs::Point> offset_vertices;
    geometry_msgs::Point offset_point;
    for (ClipperLib::IntPoint int_point : solution[0]) {
        offset_point.x = static_cast<double>(int_point.X)/scale_factor;
        offset_point.y = static_cast<double>(int_point.Y)/scale_factor;
        offset_vertices.push_back(offset_point);
    }
    return Polygon(offset_vertices);
}

/**
* @brief      Adds an obstacle to the existing obstacle space
*
* @param[in]  Polygon
*/
void Map::addObstacle(Polygon new_poly) {
    obstacles_.push_back(new_poly);
    Polygon offset_new_poly = offsetPolygon(new_poly);
    offset_obstacles_.push_back(offset_new_poly);
}

