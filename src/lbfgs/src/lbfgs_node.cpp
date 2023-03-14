#include <ros/ros.h>
#include <lbfgs/Optdata.h>
#include <bspline_opt/my_optimizer.h>


ego_planner::MyOptimizer myoptimizer;

bool Process_callback(lbfgs::Optdata::Request &request,
    lbfgs::Optdata::Response &response)
{
    cout << typeid(request.interval).name() << endl;
    ROS_INFO("Process_callback is working");
    ROS_INFO("intervel get data is %f", request.interval);
    int variable_num_ = request.variable_num_;
    double q[variable_num_];
    for (int i = 0; i < variable_num_; ++i)
        {
        q[i] = request.qes[i];
        }
    double final_cost = request.final_cost;
    // data transform
    Eigen::MatrixXd points(3, request.points.size());
    for (size_t i = 0; i < request.points.size(); ++i)
    {
      points(0, i) = request.points[i].x;
      points(1, i) = request.points[i].y;
      points(2, i) = request.points[i].z;
    }
    ego_planner::ControlPoints cps;
    cps.points = points;
    Eigen::Vector3d base_point;
    for (int i = 0; i<request.base_point.size(); ++i)
    {
      base_point[0] = request.base_point[i].x;
      base_point[1] = request.base_point[i].y;
      base_point[2] = request.base_point[i].z;
      cps.base_point[i].push_back(base_point);
    }
    Eigen::Vector3d direction;
    for (int i = 0; i<request.direction.size(); ++i)
    {
      direction[0] = request.direction[i].x;
      direction[1] = request.direction[i].y;
      direction[2] = request.direction[i].z;
      cps.direction[i].push_back(direction);
    }
    int fo = request.fo;
    ROS_INFO("wait for setparam, fo is %d", request.fo);
    // set my optimizer
    myoptimizer.setparam(fo,request.interval,request.ord,request.l1,request.l2,request.nl2,request.l3, request.mv, request.ma, request.in, cps);
    ROS_INFO("wait for processing");
    int result = myoptimizer.Processing(variable_num_, q, &final_cost);
    ROS_INFO("wait for sendback");

    // settings for sendback
    response.variable_num_ = variable_num_;
    response.qes.reserve(variable_num_);
    for (int i = 0; i < variable_num_; ++i)
    {
      response.qes.push_back(q[i]);
    }
    ROS_INFO("variable_num_ is [%d]", variable_num_);
    response.final_cost = final_cost;
    myoptimizer.setpubparams(response);
    ROS_INFO("variable_num_ in request is [%d]", response.variable_num_);
      // response_pub_->publish(response);
    response.result = result; 
    return true;
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "lbfgs_node");
  ros::NodeHandle nh("~");

  ROS_INFO("LBFGS node is on");


  
  ros::ServiceServer server = nh.advertiseService("/lbfgs/Optdata", Process_callback);


//   ros::Duration(1.0) ep();
  ros::spin();

  return 0;
}

