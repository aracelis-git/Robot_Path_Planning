/**
 * @authors   Aldrin Racelis
 * @copyright Aldrin Racelis (c)2017
 * @file      test.cpp
 * @brief     Runs simple tests on the reinforcement learning (RL) environments
 */

#include <ros/ros.h>
#include <ros/service_client.h>
#include <corridor.h>
#include <gtest/gtest.h>

TEST(TESTSuite, checkClass) {
  Random rng;
  rng = Random(1+1);
  Corridor thistest(rng, true);
  EXPECT_EQ(thistest.apply(1),-1);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
