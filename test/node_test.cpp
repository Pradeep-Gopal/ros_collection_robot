#include <gtest/gtest.h>
#include "../include/node.h"

TEST(nodeConstructor,ghf_test) {
    Node node;
    EXPECT_EQ(node.g,0);
    EXPECT_EQ(node.g,0);
    EXPECT_EQ(node.g,0);
}

TEST(OperatorTest,equals_operator_test) {
    Node node1;
    Node node2;
    node1.position.x = 5;
    node1.position.y = 6;
    node2.position.x = 5.0001;
    node2.position.y = 6.0001;
    EXPECT_EQ(node1==node2,1);
}

TEST(OperatorTest2,equals_operator_test2) {
Node node1;
Node node2;
node1.position.x = 5;
node1.position.y = 6;
node2.position.x = 9.01;
node2.position.y = 11.001;
EXPECT_EQ(node1==node2,0);
}


