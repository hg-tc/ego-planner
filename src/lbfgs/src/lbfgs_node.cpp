#include <ros/ros.h>
#include <lbfgs/Optdata.h>
#include <bspline_opt/my_optimizer.h>




bool Process_callback(lbfgs::Optdata::Request &request,
    lbfgs::Optdata::Response &response)
{
    ego_planner::MyOptimizer myoptimizer;
    cout << typeid(request.interval).name() << endl;
    ROS_INFO("Process_callback is working");
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
    cps.clearance = request.clearance;
    cps.resize(request.size);
    cps.points = points;
    int count = 0;
    for (int i = 0; i<request.size; ++i)
    {
      for (int j = 0;j<request.weightb[i];++j)
      {

        Eigen::Vector3d base_point;

        base_point[0] = request.base_point[count].x;
        base_point[1] = request.base_point[count].y;
        base_point[2] = request.base_point[count].z;

        cps.base_point[i].push_back(base_point);
        count++;

      }
    }
    count = 0;

    
    for (int i = 0; i<request.size; ++i)
    {
      for (int j = 0;j<request.weightd[i];++j)
      {
        Eigen::Vector3d direction;
        direction[0] = request.direction[count].x;
        direction[1] = request.direction[count].y;
        direction[2] = request.direction[count].z;
        cps.direction[i].push_back(direction);
        count++;
      }
    }
    count = 0;
    int fo = request.fo;

    // set my optimizer

    myoptimizer.setparam(fo,request.interval,request.ord,request.l1,request.l2,request.nl2,request.l3, request.mv, request.ma, request.in, cps);
    int result = myoptimizer.Processing(variable_num_, q, &final_cost);


    // ***********************settings for sendback**************************
    response.variable_num_ = variable_num_;
    response.qes.reserve(variable_num_);
    for (int i = 0; i < variable_num_; ++i)
    {
      response.qes.push_back(q[i]);
    }

    response.final_cost = final_cost;
    
    myoptimizer.setpubparams(response);
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

