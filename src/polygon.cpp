/**
 * @file polygon.cpp
 * @author Pradeep Gopal, Justin Albrecht Govind Ajith Kumar
 * @copyright MIT License
 * @brief Source file for the Polygon Class
 * This is the Source file for the polygon class.
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

#include "../include/polygon.h"
#include "../include/line.h"

/**
 * @brief      User defined constructor
 *
 * @param[in]  vertices  The vertices of the polygon
 */
Polygon::Polygon(std::vector<geometry_msgs::Point> vertices)
:vertices_{vertices} {
    n_ =  vertices_.size();
    calculateCentroid();
    int j;
    for (int i = 0; i < n_; i++) {
        if (i+1 == n_)
            j = 0;
        else
            j = i+1;
        lines_.push_back(Line(vertices_[i], vertices_[j], centroid_));
    }
}

/**
 * @brief      Gets the vertices.
 *
 * @return     The vertices.
 */
std::vector<geometry_msgs::Point> Polygon::getVertices() {
    return vertices_;
}

/**
 * @brief      Calculates the centroid.
 */
void Polygon::calculateCentroid() {
    double centroidX = 0, centroidY = 0;
    double det = 0;
    unsigned int j = 0;

    for (unsigned int i = 0; i < n_; i++) {
        double tempDet = 0;
        if (i + 1 == n_)
            j = 0;
        else
            j = i + 1;

        tempDet = vertices_[i].x * vertices_[j].y
         - vertices_[j].x*vertices_[i].y;
        det += tempDet;

        centroidX += (vertices_[i].x + vertices_[j].x)*tempDet;
        centroidY += (vertices_[i].y + vertices_[j].y)*tempDet;
    }

    centroidX /= 3*det;
    centroidY /= 3*det;

    centroid_.x = centroidX;
    centroid_.y = centroidY;
}

/**
 * @brief      Checks if a coordinate is inside the polygon
 *
 * @param[in]  coord  The coordinate
 *
 * @return     Boolean of the presence (true or false)
 */
bool Polygon::insideObject(geometry_msgs::Point coord) {
    for (auto line : lines_) {
        if ((line.a*coord.x) + (line.b*coord.y) + line.c < 0)
            return false;
    }
    return true;
}

/**
 * @brief      Gets the centroid.
 *
 * @return     The centroid.
 */
geometry_msgs::Point Polygon::getCentroid() {
    return centroid_;
}
