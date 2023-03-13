#include <ros/ros.h>

#include <plan_manage/lbfgs_manager.h>

using namespace ego_planner;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "lbfgs");
  ros::NodeHandle nh("~");

  ROS_INFO("LBFGS node is on");
  LBFGSManager lbfgs;
  lbfgs.init(nh);


  ros::Duration(1.0).sleep();
  ros::spin();

  return 0;
}

