#include "../include/line.h"


Line::Line(geometry_msgs::Point point_1, geometry_msgs::Point point_2, geometry_msgs::Point test_point){
    point_1_ = point_1;
    point_2_ = point_2;
    test_point_ = test_point;
    calculateCoefficients();
}
void Line::calculateCoefficients(){
    a = point_1_.y - point_2_.y;
    b = point_2_.x - point_1_.x;
    c = (point_1_.x*point_2_.y) - (point_2_.x - point_1_.y);

    if(((a*test_point_.x) + (b*test_point_.y) + c) < 0){
        a = -1;
        b = -1;
        c = -1;
    }

}
