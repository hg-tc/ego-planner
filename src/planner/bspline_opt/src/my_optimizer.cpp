#include "bspline_opt/my_optimizer.h"
#include <ego_planner/Optimizedata.h>
// using namespace std;

namespace ego_planner
{

void Processing(const ego_planner::Optimizedata::ConstPtr &msg)
{
  int variable_num_ = msg->variable_num_;
  double q[variable_num_];
  for (int i = 0; i < variable_num_; ++i)
    {
      q[i] = msg->qes[i];
    }
  double final_cost = msg->final_cost;
  lbfgs::lbfgs_parameter_t lbfgs_params;
  lbfgs::lbfgs_load_default_parameters(&lbfgs_params);
  lbfgs_params.mem_size = 16;
  lbfgs_params.max_iterations = 200;
  lbfgs_params.g_epsilon = 0.01;
//   lbfgs_params = 
//   int result = lbfgs::lbfgs_optimize(variable_num_, q, &final_cost, costFunctionRebound, NULL, earlyExit, this, &lbfgs_params);
}

void MyOptimizer::calcSmoothnessCost(const Eigen::MatrixXd &q, double &cost,
                                        Eigen::MatrixXd &gradient, bool falg_use_jerk/* = true*/)
{

    cost = 0.0;

    if (falg_use_jerk)
    {
        Eigen::Vector3d jerk, temp_j;

        for (int i = 0; i < q.cols() - 3; i++)
        {
        /* evaluate jerk */
        jerk = q.col(i + 3) - 3 * q.col(i + 2) + 3 * q.col(i + 1) - q.col(i);
        cost += jerk.squaredNorm();
        temp_j = 2.0 * jerk;
        /* jerk gradient */
        gradient.col(i + 0) += -temp_j;
        gradient.col(i + 1) += 3.0 * temp_j;
        gradient.col(i + 2) += -3.0 * temp_j;
        gradient.col(i + 3) += temp_j;
        }
    }
    else
    {
        Eigen::Vector3d acc, temp_acc;

        for (int i = 0; i < q.cols() - 2; i++)
        {
        /* evaluate acc */
        acc = q.col(i + 2) - 2 * q.col(i + 1) + q.col(i);
        cost += acc.squaredNorm();
        temp_acc = 2.0 * acc;
        /* acc gradient */
        gradient.col(i + 0) += temp_acc;
        gradient.col(i + 1) += -2.0 * temp_acc;
        gradient.col(i + 2) += temp_acc;
        }
    }
}
void MyOptimizer::calcDistanceCostRebound(const Eigen::MatrixXd &q, double &cost,
                                                Eigen::MatrixXd &gradient, int iter_num, double smoothness_cost)
{
cost = 0.0;
int end_idx = q.cols() - order_;
double demarcation = cps_.clearance;
double a = 3 * demarcation, b = -3 * pow(demarcation, 2), c = pow(demarcation, 3);

force_stop_type_ = DONT_STOP;
// if (iter_num > 3 && smoothness_cost / (cps_.size - 2 * order_) < 0.1) // 0.1 is an experimental value that indicates the trajectory is smooth enough.
// {
//     check_collision_and_rebound();
// }

/*** calculate distance cost and gradient ***/
for (auto i = order_; i < end_idx; ++i)
{
    for (size_t j = 0; j < cps_.direction[i].size(); ++j)
    {
    double dist = (cps_.points.col(i) - cps_.base_point[i][j]).dot(cps_.direction[i][j]);
    double dist_err = cps_.clearance - dist;
    Eigen::Vector3d dist_grad = cps_.direction[i][j];

    if (dist_err < 0)
    {
        /* do nothing */
    }
    else if (dist_err < demarcation)
    {
        cost += pow(dist_err, 3);
        gradient.col(i) += -3.0 * dist_err * dist_err * dist_grad;
    }
    else
    {
        cost += a * dist_err * dist_err + b * dist_err + c;
        gradient.col(i) += -(2.0 * a * dist_err + b) * dist_grad;
    }
    }
}
}

  void MyOptimizer::calcFeasibilityCost(const Eigen::MatrixXd &q, double &cost,
                                             Eigen::MatrixXd &gradient)
  {
    cost = 0.0;
    /* abbreviation */
    double ts, /*vm2, am2, */ ts_inv2;
    // vm2 = max_vel_ * max_vel_;
    // am2 = max_acc_ * max_acc_;

    ts = bspline_interval_;
    ts_inv2 = 1 / ts / ts;

    /* velocity feasibility */
    for (int i = 0; i < q.cols() - 1; i++)
    {
      Eigen::Vector3d vi = (q.col(i + 1) - q.col(i)) / ts;

      //cout << "temp_v * vi=" ;
      for (int j = 0; j < 3; j++)
      {
        if (vi(j) > max_vel_)
        {
          // cout << "fuck VEL" << endl;
          // cout << vi(j) << endl;
          cost += pow(vi(j) - max_vel_, 2) * ts_inv2; // multiply ts_inv3 to make vel and acc has similar magnitude

          gradient(j, i + 0) += -2 * (vi(j) - max_vel_) / ts * ts_inv2;
          gradient(j, i + 1) += 2 * (vi(j) - max_vel_) / ts * ts_inv2;
        }
        else if (vi(j) < -max_vel_)
        {
          cost += pow(vi(j) + max_vel_, 2) * ts_inv2;

          gradient(j, i + 0) += -2 * (vi(j) + max_vel_) / ts * ts_inv2;
          gradient(j, i + 1) += 2 * (vi(j) + max_vel_) / ts * ts_inv2;
        }
        else
        {
          /* code */
        }
      }
    }

    /* acceleration feasibility */
    for (int i = 0; i < q.cols() - 2; i++)
    {
      Eigen::Vector3d ai = (q.col(i + 2) - 2 * q.col(i + 1) + q.col(i)) * ts_inv2;

      //cout << "temp_a * ai=" ;
      for (int j = 0; j < 3; j++)
      {
        if (ai(j) > max_acc_)
        {
          // cout << "fuck ACC" << endl;
          // cout << ai(j) << endl;
          cost += pow(ai(j) - max_acc_, 2);

          gradient(j, i + 0) += 2 * (ai(j) - max_acc_) * ts_inv2;
          gradient(j, i + 1) += -4 * (ai(j) - max_acc_) * ts_inv2;
          gradient(j, i + 2) += 2 * (ai(j) - max_acc_) * ts_inv2;
        }
        else if (ai(j) < -max_acc_)
        {
          cost += pow(ai(j) + max_acc_, 2);

          gradient(j, i + 0) += 2 * (ai(j) + max_acc_) * ts_inv2;
          gradient(j, i + 1) += -4 * (ai(j) + max_acc_) * ts_inv2;
          gradient(j, i + 2) += 2 * (ai(j) + max_acc_) * ts_inv2;
        }
        else
        {
          /* code */
        }
      }
      //cout << endl;
    }
    }
void MyOptimizer::combineCostRebound(const double *x, double *grad, double &f_combine, const int n)
  {

    memcpy(cps_.points.data() + 3 * order_, x, n * sizeof(x[0]));

    /* ---------- evaluate cost and gradient ---------- */
    double f_smoothness, f_distance, f_feasibility;

    Eigen::MatrixXd g_smoothness = Eigen::MatrixXd::Zero(3, cps_.size);
    Eigen::MatrixXd g_distance = Eigen::MatrixXd::Zero(3, cps_.size);
    Eigen::MatrixXd g_feasibility = Eigen::MatrixXd::Zero(3, cps_.size);

    calcSmoothnessCost(cps_.points, f_smoothness, g_smoothness);
    calcDistanceCostRebound(cps_.points, f_distance, g_distance, iter_num_, f_smoothness);
    calcFeasibilityCost(cps_.points, f_feasibility, g_feasibility);

    f_combine = lambda1_ * f_smoothness + new_lambda2_ * f_distance + lambda3_ * f_feasibility;
    //printf("origin %f %f %f %f\n", f_smoothness, f_distance, f_feasibility, f_combine);

    Eigen::MatrixXd grad_3D = lambda1_ * g_smoothness + new_lambda2_ * g_distance + lambda3_ * g_feasibility;
    memcpy(grad, grad_3D.data() + 3 * order_, n * sizeof(grad[0]));
  }

double MyOptimizer::costFunctionRebound(void *func_data, const double *x, double *grad, const int n)
  {
    MyOptimizer *opt = reinterpret_cast<MyOptimizer *>(func_data);
    double cost;
    opt->combineCostRebound(x, grad, cost, n);

    opt->iter_num_ += 1;
    return cost;
  }
int MyOptimizer::earlyExit(void *func_data, const double *x, const double *g, const double fx, const double xnorm, const double gnorm, const double step, int n, int k, int ls)
  {
    MyOptimizer *opt = reinterpret_cast<MyOptimizer *>(func_data);
    // cout << "k=" << k << endl;
    // cout << "opt->flag_continue_to_optimize_=" << opt->flag_continue_to_optimize_ << endl;
    return (opt->force_stop_type_ == STOP_FOR_ERROR || opt->force_stop_type_ == STOP_FOR_REBOUND);
  }
}

