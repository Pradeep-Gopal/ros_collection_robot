#include <gtest/gtest.h>
#include "ros/ros.h"
#include "../include/order_manager.h"

TEST(order_manager_test,checkOrderSize) {
    OrderManager manager;
    manager.generateOrder();
    int order_size = manager.getOrderSize();
    EXPECT_EQ(manager.getOrderSize(),order_size);
}