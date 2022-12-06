#include <gtest/gtest.h>
#include <stdlib.h>

#include "../include/publisher_member_function.hpp"

TEST(publisher_test, MessageCheck) {
  rclcpp::init(0, nullptr);
  MinimalPublisher pub;
  rclcpp::shutdown();
  EXPECT_EQ(pub.msg, "ENPM808X Publisher! ");
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
