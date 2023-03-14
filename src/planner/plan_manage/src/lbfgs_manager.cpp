
#include <plan_manage/lbfgs_manager.h>
#include <typeinfo>
namespace ego_planner
{

  void LBFGSManager::init(ros::NodeHandle &nh)
  {
    ROS_INFO("LBFGS node is init");
    myoptimizer.reset(new MyOptimizer);
    pub = nh.advertise<ego_planner::Optimizedata>("/result", 100);
    sub = nh.subscribe("/planning/Optimizedata", 10, &LBFGSManager::Process_callback, this);
  }

void LBFGSManager::Process_callback(const Optimizedata::ConstPtr &msg)
{

}

} // namespace ego_planner
