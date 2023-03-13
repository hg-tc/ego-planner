#include <ros/ros.h>

#include <ego_planner/Optimizedata.h>

using namespace ego_planner;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "publisher");
  ros::NodeHandle nh("~");

  ros::Publisher pub = nh.advertise<ego_planner::Optimizedata>("/planning/Optimizedata", 10);
  ego_planner::Optimizedata data;
  data.interval =0.12;
  while(ros::ok()){
    pub.publish(data);
  }


  ros::Duration(1.0).sleep();
  ros::spin();

  return 0;
}

