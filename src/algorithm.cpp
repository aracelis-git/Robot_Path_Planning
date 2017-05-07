/**
 * @author    Aldrin I. Racelis
 * @copyright Aldrin Racelis (c)2017
 * @file      algorithm.cpp
 * @brief     Class Implementation of the Algorithm Object
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <sstream>
#include "std_msgs/String.h"

class Algorithm {
 public:
  Algorithm(const ros::NodeHandle& n);
  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& dist);
  void runAlgo();

 private:
  ros::NodeHandle n_;
  ros::Subscriber scan_sub_;
  ros::Publisher twist_pub_;
  double linear_, angular_, lscale_, ascale_, range;
};

/**
 * @brief Standard Constructor
 * @param nodehandle variable name
 */
Algorithm::Algorithm(const ros::NodeHandle& n)
  : n_(n),
    linear_(0),
    angular_(0),
    lscale_(1.0),
    ascale_(1.0) {
  scan_sub_ = n_.subscribe("scan", 1000, &Algorithm::scanCallback, this);
  twist_pub_ = n_.advertise<geometry_msgs::Twist>
                 ("mobile_base/commands/velocity", 5);
}

/**
 * @brief Reads the distance from the middle point of the laser scan
 * @param message from the laser scan
 */
void Algorithm::scanCallback(const sensor_msgs::LaserScan::ConstPtr& dist) {
  range = dist->ranges[320];
}

/**
 * @brief Determines the next movement based on the range from the laser
 */
void Algorithm::runAlgo() {
  ros::spinOnce();
  if (range != range || range >= 0.2) {
    linear_ = 2.0;
    angular_ = 0.0;
  } else if (range < 0.2) {
    linear_ = 0.0;
    angular_ = 1.0;
  } else {
    linear_ = 0.0;
    angular_ = 0.0;
  }

  geometry_msgs::Twist twist;
  twist.angular.z = ascale_*angular_;
  twist.linear.x = lscale_*linear_;
  twist_pub_.publish(twist);
}

/**
 * @brief The main program that runs the algorithm
 */
int main(int argc, char** argv) {
  ros::init(argc, argv, "walker_algorithm");
  Algorithm walker_algorithm(ros::NodeHandle(""));
  while (ros::ok()) {
    walker_algorithm.runAlgo();
  }

  return(0);
}





