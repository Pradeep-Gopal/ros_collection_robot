#include <gtest/gtest.h>
#include "../include/order_manager.h"

TEST(generateOrder,orderSizeCheck) {
    ros::NodeHandle nh;
    OrderManager manager(nh);
    manager.generateOrder();
    EXPECT_EQ(manager.getOrder().size(),manager.getOrderSize());
    EXPECT_EQ(manager.getCubes().size(),manager.getTotalCubes());
}

TEST(getTotalCubes,totalCubeTest) {
    ros::NodeHandle nh;
    OrderManager manager(nh);
    EXPECT_EQ(manager.getTotalCubes(),8);
}

TEST(getOrderSize,OrderSizeTest) {
    ros::NodeHandle nh;
    OrderManager manager(nh);
    EXPECT_EQ(manager.getOrderSize(),4);
}

TEST(getOrder,spawnandOrderCheck) {
    ros::NodeHandle nh;
    OrderManager manager(nh);
    manager.generateOrder();
    manager.spawnCubes();
    std::vector<char> cubes;
    cubes = manager.getCubes();
    EXPECT_EQ(cubes.size(),manager.getTotalCubes());
}

TEST(getCubes,spawnandCubeCheck) {
    ros::NodeHandle nh;
    OrderManager manager(nh);
    manager.generateOrder();
    manager.spawnCubes();
    std::vector<char> orders;
    orders = manager.getOrder();
    EXPECT_EQ(orders.size(),manager.getOrderSize());
}