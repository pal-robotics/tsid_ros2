#ifndef TSID_CONTROLLERS__FORCE_DERIVATIVE_FILTER_HPP_
#define TSID_CONTROLLERS__FORCE_DERIVATIVE_FILTER_HPP_

#include <array>
#include <Eigen/Dense>

namespace tsid_controllers
{

class ForceDerivativeFilter
{
public:
  ForceDerivativeFilter(double alpha)
  : alpha_(alpha)
    , previous_ema_(Eigen::VectorXd::Zero(6))
    , previous_derivative_(Eigen::VectorXd::Zero(6)) {}

  Eigen::VectorXd filter(
    const std::array<double, 3> & forces, const std::array<double,
    3> & torques, double dt)
  {
    Eigen::VectorXd current_value(6);
    current_value << forces[0], forces[1], forces[2], torques[0], torques[1], torques[2];

    previous_ema_ = alpha_ * current_value + (1.0 - alpha_) * previous_ema_;
    previous_derivative_ = (previous_ema_ - current_value) / dt;

    return previous_ema_;
  }

  Eigen::VectorXd getWrench() const
  {
    return previous_ema_;
  }

  Eigen::VectorXd getWrenchDerivative() const
  {
    return previous_derivative_;
  }

private:
  double alpha_;
  Eigen::VectorXd previous_ema_;
  Eigen::VectorXd previous_derivative_;
};

}  // namespace tsid_controllers

#endif // TSID_CONTROLLERS_FORCE_DERIVATIVE_FILTER_HPP
