
#include <plan_manage/lbfgs_manager.h>

namespace ego_planner
{

  void LBFGSManager::init(ros::NodeHandle &nh)
  {
    ROS_INFO("LBFGS node is init");
    pub = nh.advertise<ego_planner::Optimizedata>("/result", 10);
    sub = nh.subscribe("/planning/Optimizedata", 10, &LBFGSManager::Process_callback, this);
  }

void LBFGSManager::Process_callback(const Optimizedata::ConstPtr &msg)
{
    ROS_INFO("Process_callback is working");
    ROS_INFO("intervel get data is %f", msg->interval);
    int variable_num_ = msg->variable_num_;
    double q[variable_num_];
    for (int i = 0; i < variable_num_; ++i)
        {
        q[i] = msg->qes[i];
        }
    double final_cost = msg->final_cost;
    ego_planner::MyOptimizer::Ptr myoptimizer;
    // data transform
    Eigen::MatrixXd points(3, msg->points.size());
    for (size_t i = 0; i < msg->points.size(); ++i)
    {
      points(0, i) = msg->points[i].x;
      points(1, i) = msg->points[i].y;
      points(2, i) = msg->points[i].z;
    }
    ego_planner::ControlPoints cps;
    cps.points = points;
    Eigen::Vector3d base_point;
    for (int i = 0; i<msg->base_point.size(); ++i)
    {
      base_point[0] = msg->base_point[i].x;
      base_point[1] = msg->base_point[i].y;
      base_point[2] = msg->base_point[i].z;
      cps.base_point[i].push_back(base_point);
    }
    Eigen::Vector3d direction;
    for (int i = 0; i<msg->direction.size(); ++i)
    {
      direction[0] = msg->direction[i].x;
      direction[1] = msg->direction[i].y;
      direction[2] = msg->direction[i].z;
      cps.direction[i].push_back(direction);
    }
    int fo = msg->fo;
    ROS_INFO("wait for setparam, fo is %d", msg->fo);
    // set my optimizer
    myoptimizer->setparam(fo,msg->interval,msg->ord,msg->l1,msg->l2,msg->nl2,msg->l3, msg->mv, msg->ma, msg->in, cps);
    ROS_INFO("wait for processing");
    int result = myoptimizer->Processing(variable_num_, q, &final_cost);
    ROS_INFO("wait for sendback");
    // settings for sendback
    ego_planner::Optimizedata Optimizedata;
    Optimizedata.variable_num_ = variable_num_;
    Optimizedata.qes.reserve(variable_num_);
    for (int i = 0; i < variable_num_; ++i)
    {
      Optimizedata.qes.push_back(q[i]);
    }
    ROS_INFO("variable_num_ is [%d]", variable_num_);
    Optimizedata.final_cost = final_cost;
    myoptimizer->setpubparams(Optimizedata);
    ROS_INFO("variable_num_ in msg is [%d]", Optimizedata.variable_num_);
      // Optimizedata_pub_->publish(Optimizedata);
    Optimizedata.result = result; 
    pub.publish(Optimizedata);
}

} // namespace ego_planner
