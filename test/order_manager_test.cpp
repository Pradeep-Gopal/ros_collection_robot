#include <gtest/gtest.h>
#include "ros/ros.h"
#include "../include/order_manager.h"

TEST(generateOrder_test,checkOrderSize) {
    OrderManager manager;
    manager.generateOrder();
    EXPECT_EQ(manager.getOrderSize(),manager.getOrder().size());
}
