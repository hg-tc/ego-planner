#include <ros/ros.h>
#include <std_msgs/String.h>
#include "bspline_opt/my_optimizer.h"
#include <ego_planner/Optimizedata.h>


void Process_callback(const ego_planner::Optimizedata::ConstPtr &msg)
{
    ego_planner::MyOptimizer::Ptr myoptimizer;

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "lbfgs");
  ros::NodeHandle nh("~");

  
  ROS_INFO("lbfgs node is on");

  ros::Publisher pub = nh.advertise<ego_planner::Optimizedata>("result", 10);
  ros::Subscriber sub = nh.subscribe("data", 10, Process_callback);
  ego_planner::Optimizedata msg;
  while (ros::ok())
  {
    // msg
    pub.publish(msg);
  }


//   ros::Duration(1.0).sleep();
  ros::spin();

  return 0;
}

