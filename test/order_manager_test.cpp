#include <gtest/gtest.h>
#include "../include/order_manager.h"

TEST(order_size_test,checkOrderSize) {
    OrderManager manager;
    manager.generateOrder();
    EXPECT_EQ(manager.getOrderSize(),4);
}

TEST(cube_test,checkTotalCubes) {
    OrderManager manager;
    manager.spawnCubes();
    EXPECT_EQ(manager.getTotalCubes(),8);
}

TEST(cube_vec_test,checkTotalCubes) {
    OrderManager manager;
    std::vector<char> order;
    order = manager.getOrder();
    EXPECT_EQ(order.size(),4);
}

TEST(order_vec_test,checkTotalCubes) {
    OrderManager manager;
    std::vector<char> cubes;
    cubes = manager.getCubes();
    EXPECT_EQ(cubes.size(),8);
}