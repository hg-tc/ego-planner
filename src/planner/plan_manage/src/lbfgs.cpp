#include <ros/ros.h>
#include <std_msgs/String.h>
#include "bspline_opt/bspline_optimizer.h"
#include <ego_planner/Optimizedata.h>




void Processing(const ego_planner::Optimizedata::ConstPtr &msg)
{
//   d 
//   variable_num_ = 
//   q = 
//   final_cost = 
//   lbfgs_params = 
//   int result = lbfgs::lbfgs_optimize(variable_num_, q, &final_cost, BsplineOptimizer::costFunctionRebound, NULL, BsplineOptimizer::earlyExit, this, &lbfgs_params);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "lbfgs");
  ros::NodeHandle nh("~");

  
  ROS_INFO("lbfgs node is on");

  ros::Publisher pub = nh.advertise<ego_planner::Optimizedata>("result", 10);
  ros::Subscriber sub = nh.subscribe("data", 10, Processing);
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

