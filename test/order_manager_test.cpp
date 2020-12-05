#include <gtest/gtest.h>
#include "../include/order_manager.h"

TEST(generateOrder,orderSizeCheck) {
    OrderManager manager;
    manager.generateOrder();
    EXPECT_EQ(manager.getOrder().size(),manager.getOrderSize());
    EXPECT_EQ(manager.getCubes().size(),manager.getTotalCubes());
}