#ifndef _MY_OPTIMIZER_H_
#define _MY_OPTIMIZER_H_

#include <Eigen/Eigen>
#include <path_searching/dyn_a_star.h>
#include <bspline_opt/uniform_bspline.h>
#include <plan_env/grid_map.h>
#include <ros/ros.h>
#include "bspline_opt/lbfgs.hpp"
#include <lbfgs/Optdata.h>
#include <bspline_opt/bspline_optimizer.h>
// Gradient and elasitc band optimization

// Input: a signed distance field and a sequence of points
// Output: the optimized sequence of points
// The format of points: N x 3 matrix, each row is a point
namespace ego_planner
{
  class MyOptimizer
  {

  public:
    MyOptimizer() {}
    ~MyOptimizer() {}

    
    int Processing(int variable_num_, double *q, double *final_cost);

  private:

    enum FORCE_STOP_OPTIMIZE_TYPE
    {
      DONT_STOP,
      STOP_FOR_REBOUND,
      STOP_FOR_ERROR
    } force_stop_type_;


    double bspline_interval_; 
    int order_;                    // bspline degree
    double lambda1_;               // jerk smoothness weight
    double lambda2_, new_lambda2_; // distance weight
    double lambda3_;               // feasibility weight
    double max_vel_, max_acc_; // dynamic limits
    int iter_num_;                  // iteration of the solver
    ControlPoints cps_;

    // q contains all control points
    void calcSmoothnessCost(const Eigen::MatrixXd &q, double &cost,
                            Eigen::MatrixXd &gradient, bool falg_use_jerk = true);
    void calcFeasibilityCost(const Eigen::MatrixXd &q, double &cost,
                             Eigen::MatrixXd &gradient);
    void calcDistanceCostRebound(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient, int iter_num, double smoothness_cost);


    static int earlyExit(void *func_data, const double *x, const double *g, const double fx, const double xnorm, const double gnorm, const double step, int n, int k, int ls);
    static double costFunctionRebound(void *func_data, const double *x, double *grad, const int n);
    static double costFunctionRefine(void *func_data, const double *x, double *grad, const int n);
    void combineCostRebound(const double *x, double *grad, double &f_combine, const int n);
    void combineCostRefine(const double *x, double *grad, double &f_combine, const int n);
  public:
    typedef unique_ptr<MyOptimizer> Ptr;

    void setparam(int f, double interval, int ord, 
      double l1, double l2, double nl2, double l3,
      double mv, double ma, int in, ControlPoints cps)
    {
      ROS_INFO("start setparam");
      // ROS_INFO("f is [%d]", f);
      // ROS_INFO("interval is [%f]", interval);
      
      if(f == 0) force_stop_type_ = DONT_STOP;
      else if(f == 1) force_stop_type_ = STOP_FOR_REBOUND;
      else force_stop_type_ = STOP_FOR_ERROR;
      // ROS_INFO("after set force");
      bspline_interval_ = interval;
      order_ = ord;
      lambda1_ = l1;lambda2_ = l2;new_lambda2_ = nl2;lambda3_ = l3;
      // ROS_INFO("before set cps");
      max_vel_ = mv;max_acc_ = ma;iter_num_ = in; cps_ = cps;
      // ROS_INFO("finish setparam");
    }
    void setpubparams(lbfgs::Optdata::Response &msg)
    {
      msg.fo = force_stop_type_;msg.interval = bspline_interval_;
      msg.ord = order_; msg.in = iter_num_; msg.l1 = lambda1_;
      msg.l2 = lambda2_; msg.nl2 = new_lambda2_; msg.l3 = lambda3_;
      msg.mv = max_vel_; msg.ma = max_acc_;
      for (int i = 0; i < cps_.size; ++i)
      {
        geometry_msgs::Point pt;
        pt.x = cps_.points(0, i);
        pt.y = cps_.points(1, i);
        pt.z = cps_.points(2, i);
        msg.points.push_back(pt);
        int j= 0;
        for(j = 0;j<cps_.base_point[i].size();j++)
        {
          geometry_msgs::Point pt2;
          pt2.x = cps_.base_point[i][j](0);
          pt2.y = cps_.base_point[i][j](1);
          pt2.z = cps_.base_point[i][j](2);
          msg.base_point.push_back(pt2);          
        }
        msg.weightb.push_back(j);

        for(j = 0;j<cps_.direction[i].size();j++)
        {
          geometry_msgs::Point pt3;
          pt3.x = cps_.direction[i][j](0);
          pt3.y = cps_.direction[i][j](1);
          pt3.z = cps_.direction[i][j](2);
          msg.direction.push_back(pt3);
        }
        msg.weightd.push_back(j);
      }

    }
  };
  
} // namespace ego_planner
#endif