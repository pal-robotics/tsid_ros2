#ifndef __invdyn_task_cartesian_velocity_hpp__
#define __invdyn_task_cartesian_velocity_hpp__

#include "tsid/tasks/task-motion.hpp"
#include "tsid/trajectories/trajectory-base.hpp"
#include "tsid/math/constraint-equality.hpp"

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>

namespace tsid
{
namespace tasks
{

class TaskCartesianVelocity : public TaskMotion
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef math::Index Index;
  typedef trajectories::TrajectorySample TrajectorySample;
  typedef math::Vector Vector;
  typedef math::ConstraintEquality ConstraintEquality;
  typedef pinocchio::Data Data;
  typedef pinocchio::Data::Matrix6x Matrix6x;
  typedef pinocchio::Motion Motion;
  typedef pinocchio::SE3 SE3;

  TaskCartesianVelocity(
    const std::string & name, RobotWrapper & robot,
    const std::string & frameName);

  virtual ~TaskCartesianVelocity() {}

  int dim() const;

  const ConstraintBase & compute(
    const double t, ConstRefVector q,
    ConstRefVector v, Data & data);

  const ConstraintBase & getConstraint() const;

  void setReference(TrajectorySample & ref);
  void setReference(const TrajectorySample & ref);
  const TrajectorySample & getReference() const;

  /** Return the desired task acceleration (after applying the specified mask).
   *  The value is expressed in local frame is the local_frame flag is true,
   *  otherwise it is expressed in a local world-oriented frame.
   */
  const Vector & getDesiredAcceleration() const;

  /** Return the task acceleration (after applying the specified mask).
   *  The value is expressed in local frame is the local_frame flag is true,
   *  otherwise it is expressed in a local world-oriented frame.
   */
  Vector getAcceleration(ConstRefVector dv) const;

  virtual void setMask(math::ConstRefVector mask);

  /** Return the position tracking error (after applying the specified mask).
   *  The error is expressed in local frame is the local_frame flag is true,
   *  otherwise it is expressed in a local world-oriented frame.
   */
  const Vector & position_error() const;

  /** Return the velocity tracking error (after applying the specified mask).
   *  The error is expressed in local frame is the local_frame flag is true,
   *  otherwise it is expressed in a local world-oriented frame.
   */
  const Vector & velocity_error() const;

  const Vector & position() const;
  const Vector & velocity() const;
  const Vector & position_ref() const;
  const Vector & velocity_ref() const;

  const Vector & Kp() const;
  const Vector & Kd() const;
  const Vector & Ki() const;
  void Kp(ConstRefVector Kp);
  void Kd(ConstRefVector Kp);
  void Ki(ConstRefVector Ki);

  Index frame_id() const;

  /**
   * @brief Specifies if the jacobian and desired acceloration should be
   * expressed in the local frame or the local world-oriented frame.
   *
   * @param local_frame If true, represent jacobian and acceloration in the
   *   local frame. If false, represent them in the local world-oriented frame.
   */
  void useLocalFrame(bool local_frame);

protected:
  std::string m_frame_name;
  Index m_frame_id;
  Motion m_p_error, m_v_error, m_a_error;
  Vector m_p_error_vec, m_v_error_vec;
  Vector m_p_error_masked_vec, m_v_error_masked_vec;
  Vector m_p, m_v;
  Vector m_p_ref, m_v_ref_vec;
  Motion m_v_ref, m_a_ref, v_prev, v_err_prev, m_a_prev;
  SE3 m_M_ref, m_wMl, pos_prev;
  Vector m_Kp;
  Vector m_Kd;
  Vector m_Ki;
  Vector m_a_des, m_a_des_masked;
  Motion m_drift;
  Vector m_drift_masked;
  Matrix6x m_J;
  Matrix6x m_J_rotated;
  ConstraintEquality m_constraint;
  TrajectorySample m_ref;
  double dt_;

  bool first = true;
  bool m_local_frame;
};

}  // namespace tasks
}  // namespace tsid

#endif  // ifndef __invdyn_task_se3_equality_hpp__
