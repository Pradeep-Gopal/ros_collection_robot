#include <gtest/gtest.h>
#include "../include/order_manager.h"

//TEST(generateOrder,orderSizeCheck) {
//    OrderManager manager;
//    manager.generateOrder();
//    EXPECT_EQ(manager.getOrder().size(),manager.getOrderSize());
//    EXPECT_EQ(manager.getCubes().size(),manager.getTotalCubes());
//}
//
//TEST(getTotalCubes,totalCubeTest) {
//    OrderManager manager;
//    EXPECT_EQ(manager.getTotalCubes(),8);
//}
//
//TEST(getOrderSize,OrderSizeTest) {
//    OrderManager manager;
//    EXPECT_EQ(manager.getOrderSize(),4);
//}
//
//TEST(getOrder,spawnandOrderCheck) {
//    OrderManager manager;
//    manager.generateOrder();
//    manager.spawnCubes();
//    std::vector<char> cubes;
//    cubes = manager.getCubes();
//    EXPECT_EQ(cubes.size(),manager.getTotalCubes());
//}
//
//TEST(getCubes,spawnandCubeCheck) {
//    OrderManager manager;
//    manager.generateOrder();
//    manager.spawnCubes();
//    std::vector<char> orders;
//    orders = manager.getOrder();
//    EXPECT_EQ(orders.size(),manager.getOrderSize());
//}