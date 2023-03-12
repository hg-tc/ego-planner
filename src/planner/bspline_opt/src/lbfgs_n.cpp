#include <ros/ros.h>
#include <std_msgs/String.h>
#include <bspline_opt/my_optimizer.h>
#include <ego_planner/Optimizedata.h>
#include <bspline_opt/bspline_optimizer.h>

ego_planner::BsplineOptimizer::Ptr bspline_optimizer_rebound_;
ego_planner::MyOptimizer::Ptr myoptimizer;
void Process_callback(const ego_planner::Optimizedata::ConstPtr &msg)
{
    int variable_num_ = msg->variable_num_;
    double q[variable_num_];
    for (int i = 0; i < variable_num_; ++i)
        {
        q[i] = msg->qes[i];
        }
    double final_cost = msg->final_cost;
    


    myoptimizer->Processing(variable_num_, q, &final_cost);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "lbfgs_n");
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

