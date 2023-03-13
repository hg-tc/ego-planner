#include <ros/ros.h>
#include <std_msgs/String.h>
#include <bspline_opt/my_optimizer.h>
#include <ego_planner/Optimizedata.h>
#include <plan_manage/lbfgs_manager.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "lbfgs_n");
  ros::NodeHandle nh("~");

  ROS_INFO("LBFGS node is on");
  ego_planner::LBFGSManager lbfgs;
  lbfgs.init(nh);


//   ros::Duration(1.0).sleep();
  ros::spin();

  return 0;
}

