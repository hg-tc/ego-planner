#ifndef _BSPLINE_OPTIMIZER_H_
#define _BSPLINE_OPTIMIZER_H_

#include <Eigen/Eigen>
#include <path_searching/dyn_a_star.h>
#include <bspline_opt/uniform_bspline.h>
#include <plan_env/grid_map.h>
#include <ros/ros.h>
#include "bspline_opt/lbfgs.hpp"
#include <lbfgs/Optdata.h>

// Gradient and elasitc band optimization

// Input: a signed distance field and a sequence of points
// Output: the optimized sequence of points
// The format of points: N x 3 matrix, each row is a point
namespace ego_planner
{

  class ControlPoints
  {
  public:
    double clearance;
    int size;
    Eigen::MatrixXd points;
    std::vector<std::vector<Eigen::Vector3d>> base_point; // The point at the statrt of the direction vector (collision point)
    std::vector<std::vector<Eigen::Vector3d>> direction;  // Direction vector, must be normalized.
    std::vector<bool> flag_temp;                          // A flag that used in many places. Initialize it everytime before using it.
    // std::vector<bool> occupancy;

    void resize(const int size_set)
    {
      size = size_set;

      base_point.clear();
      direction.clear();
      flag_temp.clear();
      // occupancy.clear();

      points.resize(3, size_set);
      base_point.resize(size);
      direction.resize(size);
      flag_temp.resize(size);
      // occupancy.resize(size);
    }
  };

  class BsplineOptimizer
  {

  public:
    BsplineOptimizer() {}
    ~BsplineOptimizer() {}

    /* main API */
    void setEnvironment(const GridMap::Ptr &env);
    void setParam(ros::NodeHandle &nh);
    Eigen::MatrixXd BsplineOptimizeTraj(const Eigen::MatrixXd &points, const double &ts,
                                        const int &cost_function, int max_num_id, int max_time_id);

    /* helper function */

    // required inputs
    void setControlPoints(const Eigen::MatrixXd &points);
    void setBsplineInterval(const double &ts);
    void setCostFunction(const int &cost_function);
    void setTerminateCond(const int &max_num_id, const int &max_time_id);

    // optional inputs
    void setGuidePath(const vector<Eigen::Vector3d> &guide_pt);
    void setWaypoints(const vector<Eigen::Vector3d> &waypts,
                      const vector<int> &waypt_idx); // N-2 constraints at most

    void optimize();

    Eigen::MatrixXd getControlPoints();

    AStar::Ptr a_star_;
    std::vector<Eigen::Vector3d> ref_pts_;

    std::vector<std::vector<Eigen::Vector3d>> initControlPoints(Eigen::MatrixXd &init_points, bool flag_first_init = true);
    bool BsplineOptimizeTrajRebound(Eigen::MatrixXd &optimal_points, double ts, ros::ServiceClient *Optdata_client); // must be called after initControlPoints()
    bool BsplineOptimizeTrajRefine(const Eigen::MatrixXd &init_points, const double ts, Eigen::MatrixXd &optimal_points);

    inline int getOrder(void) { return order_; }

  private:
    GridMap::Ptr grid_map_;

    enum FORCE_STOP_OPTIMIZE_TYPE
    {
      DONT_STOP,
      STOP_FOR_REBOUND,
      STOP_FOR_ERROR
    } force_stop_type_;

    // main input
    // Eigen::MatrixXd control_points_;     // B-spline control points, N x dim
    double bspline_interval_; // B-spline knot span
    Eigen::Vector3d end_pt_;  // end of the trajectory
    // int             dim_;                // dimension of the B-spline
    //
    vector<Eigen::Vector3d> guide_pts_; // geometric guiding path points, N-6
    vector<Eigen::Vector3d> waypoints_; // waypts constraints
    vector<int> waypt_idx_;             // waypts constraints index
                                        //
    int max_num_id_, max_time_id_;      // stopping criteria
    int cost_function_;                 // used to determine objective function
    double start_time_;                 // global time for moving obstacles

    /* optimization parameters */
    int order_;                    // bspline degree
    double lambda1_;               // jerk smoothness weight
    double lambda2_, new_lambda2_; // distance weight
    double lambda3_;               // feasibility weight
    double lambda4_;               // curve fitting

    int a;
    //
    double dist0_;             // safe distance
    double max_vel_, max_acc_; // dynamic limits

    int variable_num_;              // optimization variables
    int iter_num_;                  // iteration of the solver
    Eigen::VectorXd best_variable_; //
    double min_cost_;               //

    ControlPoints cps_;

    /* cost function */
    /* calculate each part of cost function with control points q as input */

    static double costFunction(const std::vector<double> &x, std::vector<double> &grad, void *func_data);
    void combineCost(const std::vector<double> &x, vector<double> &grad, double &cost);

    // q contains all control points
    void calcSmoothnessCost(const Eigen::MatrixXd &q, double &cost,
                            Eigen::MatrixXd &gradient, bool falg_use_jerk = true);
    void calcFeasibilityCost(const Eigen::MatrixXd &q, double &cost,
                             Eigen::MatrixXd &gradient);
    void calcDistanceCostRebound(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient, int iter_num, double smoothness_cost);
    void calcFitnessCost(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient);
    bool check_collision_and_rebound(void);

    static int earlyExit(void *func_data, const double *x, const double *g, const double fx, const double xnorm, const double gnorm, const double step, int n, int k, int ls);
    static double costFunctionRebound(void *func_data, const double *x, double *grad, const int n);
    static double costFunctionRefine(void *func_data, const double *x, double *grad, const int n);

    bool rebound_optimize(ros::ServiceClient *Optdata_client);
    bool refine_optimize();
    void combineCostRebound(const double *x, double *grad, double &f_combine, const int n);
    void combineCostRefine(const double *x, double *grad, double &f_combine, const int n);


    /* for benckmark evaluation only */
  public:
    typedef unique_ptr<BsplineOptimizer> Ptr;
    void setpubparams(lbfgs::Optdata &msg)
    {
      if(force_stop_type_ == DONT_STOP) msg.request.fo = 0;
      else if(force_stop_type_ == STOP_FOR_REBOUND) msg.request.fo = 1;
      else msg.request.fo = 2;
      msg.request.interval = bspline_interval_;

      msg.request.ord = order_; msg.request.in = iter_num_; msg.request.l1 = lambda1_;
      msg.request.l2 = lambda2_; msg.request.nl2 = new_lambda2_; msg.request.l3 = lambda3_;
      msg.request.mv = max_vel_; msg.request.ma = max_acc_;
      msg.request.clearance = cps_.clearance;
      msg.request.size = cps_.size;
    }
    void setparam(lbfgs::Optdata &msg)
    {
      if(msg.response.fo == 0) force_stop_type_ = DONT_STOP;
      else if(msg.response.fo == 1) force_stop_type_ = STOP_FOR_REBOUND;
      else force_stop_type_ = STOP_FOR_ERROR;
      bspline_interval_ = msg.response.interval;
      order_ = msg.response.ord;
      lambda1_ = msg.response.l1;lambda2_ = msg.response.l2;new_lambda2_ = msg.response.nl2;lambda3_ = msg.response.l3;
      max_vel_ = msg.response.mv;max_acc_ = msg.response.ma;iter_num_ = msg.response.in;
      
      ego_planner::ControlPoints cps;
      cps.clearance = cps_.clearance;
      cps.resize(cps_.size);
      Eigen::MatrixXd points(3, msg.response.points.size());
      for (size_t i = 0; i < msg.response.points.size(); ++i)
      {
        points(0, i) = msg.response.points[i].x;
        points(1, i) = msg.response.points[i].y;
        points(2, i) = msg.response.points[i].z;
      }
      cps.points = points;
      int count = 0;
      for (int i = 0; i<cps_.size; ++i)
      {
        for (int j = 0;j<msg.response.weightb[i];++j)
        {

          Eigen::Vector3d base_point;

          base_point[0] = msg.response.base_point[count].x;
          base_point[1] = msg.response.base_point[count].y;
          base_point[2] = msg.response.base_point[count].z;

          cps.base_point[i].push_back(base_point);
          count++;
        }
      }
      count = 0;
      for (int i = 0; i<cps_.size; ++i)
      {
        for (int j = 0;j<msg.response.weightd[i];++j)
        {
          Eigen::Vector3d direction;
          direction[0] = msg.response.direction[count].x;
          direction[1] = msg.response.direction[count].y;
          direction[2] = msg.response.direction[count].z;
          cps.direction[i].push_back(direction);
          count++;
        }
      }
      count = 0;
      cps_ = cps;
    }
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

} // namespace ego_planner
#endif