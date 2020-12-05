#include "ros/ros.h"
#include "../include/polygon.h"

Polygon::Polygon(std::vector<geometry_msgs::Point> vertices){
    vertices_ = vertices;
    n_ =  vertices_.size();
    calculateCentroid();
}
void Polygon::calculateCentroid(){
    double centroidX = 0, centroidY = 0;
    double det = 0, tempDet = 0;
    unsigned int j = 0;

    for (unsigned int i = 0; i < n_; i++)
    {
        if (i + 1 == n_)
            j = 0;
        else
            j = i + 1;

        tempDet = vertices_[i].x * vertices_[j].y - vertices_[j].x*vertices_[i].y;
        det += tempDet;

        centroidX += (vertices_[i].x + vertices_[j].x)*tempDet;
        centroidY += (vertices_[i].y + vertices_[j].y)*tempDet;
    }

    centroidX /= 3*det;
    centroidY /= 3*det;

    centroid_.x = centroidX;
    centroid_.y = centroidY;
}

bool Polygon::insideObject(geometry_msgs::Point coord){

}

geometry_msgs::Point Polygon::getCentroid(){
    return centroid_;
}
