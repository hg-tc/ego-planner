#ifndef _LBFGS_MANAGER_H_
#define _LBFGS_MANAGER_H_

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <bspline_opt/my_optimizer.h>
#include <ego_planner/Optimizedata.h>

using std::vector;

namespace ego_planner
{

  class LBFGSManager
  {

  private:

    /* ROS utils */
    ros::Subscriber sub;
    ros::Publisher pub;
    void Process_callback(const Optimizedata::ConstPtr &msg);
  public:
    LBFGSManager(/* args */)
    {
    }
    ~LBFGSManager()
    {
    }

    void init(ros::NodeHandle &nh);


  };

} // namespace ego_planner

#endif