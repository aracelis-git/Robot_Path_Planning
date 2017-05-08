/**
 * @authors   Aldrin Racelis
 * @copyright Aldrin Racelis (c)2017
 * @file      test.cpp
 * @brief     Runs simple tests on the reinforcement learning (RL) environments
 */

#include <ros/ros.h>
#include <ros/service_client.h>
#include <gtest/gtest.h>

std::shared_ptr<ros::NodeHandle> nh;

TEST(TESTSuite, checkMessage)
{
 // ros::ServiceClient client = nh->serviceClient<Robot_Path_Planning::algorithm>(
//      "algorithm");
 // bool exists(client.waitForExistence(ros::Duration(1)));
//  EXPECT_TRUE(exists);


 // EXPECT_EQ(srv.response.sum, 5+8);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "algorithm");
  nh.reset(new ros::NodeHandle);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
