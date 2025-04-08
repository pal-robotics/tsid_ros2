//
// Copyright (c) 2017 CNRS
//
// This file is part of tsid
// tsid is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
// tsid is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// General Lesser Public License for more details. You should have
// received a copy of the GNU Lesser General Public License along with
// tsid If not, see
// <http://www.gnu.org/licenses/>.
//

#include "tsid_controllers/tasks/task-vel-joint.hpp"
#include "tsid/robots/robot-wrapper.hpp"
#include <pinocchio/algorithm/joint-configuration.hpp>

namespace tsid
{
namespace tasks
{
using namespace math;
using namespace trajectories;
using namespace pinocchio;

TaskJointVel::TaskJointVel(const std::string & name, RobotWrapper & robot, const double & dt)
: TaskMotion(name, robot),
  m_ref(robot.nq_actuated(), robot.na()),
  m_constraint(name, robot.na(), robot.nv()),
  dt_(dt)
{

  m_p_error.setZero(robot.na());
  v_err_prev.setZero(robot.na());
  m_ref_q_augmented = pinocchio::neutral(robot.model());
  m_Kp.setZero(robot.na());
  m_Kd.setZero(robot.na());
  Vector m = Vector::Ones(robot.na());
  setMask(m);
}

const Vector & TaskJointVel::mask() const {return m_mask;}

void TaskJointVel::mask(const Vector & m)
{
  // std::cerr<<"The method TaskJointVel::mask is deprecated. Use
  // TaskJointVel::setMask instead.\n";
  return setMask(m);
}

void TaskJointVel::setMask(ConstRefVector m)
{
  PINOCCHIO_CHECK_INPUT_ARGUMENT(
    m.size() == m_robot.na(),
    "The size of the mask needs to equal " + std::to_string(m_robot.na()));
  m_mask = m;
  const Vector::Index dim = static_cast<Vector::Index>(m.sum());
  Matrix S = Matrix::Zero(dim, m_robot.nv());
  m_activeAxes.resize(dim);
  unsigned int j = 0;
  for (unsigned int i = 0; i < m.size(); i++) {
    if (std::abs(m(i)) > std::numeric_limits<double>::epsilon()) {
      PINOCCHIO_CHECK_INPUT_ARGUMENT(
        std::abs(m(i) - 1.0) < std::numeric_limits<double>::epsilon(),
        "Valid mask values are either 0.0 or 1.0 received: " +
        std::to_string(m(i)));
      S(j, m_robot.nv() - m_robot.na() + i) = 1.0;
      m_activeAxes(j) = i;
      j++;
    }
  }
  m_constraint.resize((unsigned int)dim, m_robot.nv());
  m_constraint.setMatrix(S);
}

int TaskJointVel::dim() const {return (int)m_mask.sum();}

const Vector & TaskJointVel::Kp() {return m_Kp;}

const Vector & TaskJointVel::Kd() {return m_Kd;}

const Vector & TaskJointVel::Ki() {return m_Ki;}

void TaskJointVel::Kp(ConstRefVector Kp)
{
  PINOCCHIO_CHECK_INPUT_ARGUMENT(
    Kp.size() == m_robot.na(),
    "The size of the Kp vector needs to equal " +
    std::to_string(m_robot.na()));
  m_Kp = Kp;
}

void TaskJointVel::Kd(ConstRefVector Kd)
{
  PINOCCHIO_CHECK_INPUT_ARGUMENT(
    Kd.size() == m_robot.na(),
    "The size of the Kd vector needs to equal " +
    std::to_string(m_robot.na()));
  m_Kd = Kd;
}

void TaskJointVel::Ki(ConstRefVector Ki)
{
  PINOCCHIO_CHECK_INPUT_ARGUMENT(
    Ki.size() == m_robot.na(),
    "The size of the Ki vector needs to equal " +
    std::to_string(m_robot.na()));
  m_Ki = Ki;
}

void TaskJointVel::setReference(const TrajectorySample & ref)
{
  PINOCCHIO_CHECK_INPUT_ARGUMENT(
    ref.getValue().size() == m_robot.nq_actuated(),
    "The size of the reference value vector needs to equal " +
    std::to_string(m_robot.nq_actuated()));
  PINOCCHIO_CHECK_INPUT_ARGUMENT(
    ref.getDerivative().size() == m_robot.na(),
    "The size of the reference value derivative vector needs to equal " +
    std::to_string(m_robot.na()));
  PINOCCHIO_CHECK_INPUT_ARGUMENT(
    ref.getSecondDerivative().size() == m_robot.na(),
    "The size of the reference value second derivative vector needs to "
    "equal " +
    std::to_string(m_robot.na()));
  m_ref = ref;
}

const TrajectorySample & TaskJointVel::getReference() const {return m_ref;}

const Vector & TaskJointVel::getDesiredAcceleration() const
{
  return m_a_des;
}

Vector TaskJointVel::getAcceleration(ConstRefVector dv) const
{
  return m_constraint.matrix() * dv;
}

const Vector & TaskJointVel::position_error() const {return m_p_error;}

const Vector & TaskJointVel::velocity_error() const {return m_v_error;}

const Vector & TaskJointVel::position() const {return m_p;}

const Vector & TaskJointVel::velocity() const {return m_v;}

const Vector & TaskJointVel::position_ref() const
{
  return m_ref.getValue();
}

const Vector & TaskJointVel::velocity_ref() const
{
  return m_ref.getValue();
}

const ConstraintBase & TaskJointVel::getConstraint() const
{
  return m_constraint;
}

const ConstraintBase & TaskJointVel::compute(
  const double, ConstRefVector,
  ConstRefVector v, Data &)
{
  m_ref_q_augmented.tail(m_robot.nq_actuated()) = m_ref.getValue();

  m_v = v.tail(m_robot.na());

  m_v_error = m_ref.getValue() - m_v;
  m_p_error += m_v_error * dt_;
  m_a_error = (m_v_error - v_err_prev ) / dt_;    // acc err in local world-oriented frame
  m_a_des = m_Kp.cwiseProduct(m_v_error) + //m_Ki.cwiseProduct(m_p_error) +
    m_Kd.cwiseProduct(m_a_error);


  for (unsigned int i = 0; i < m_activeAxes.size(); i++) {
    m_constraint.vector()(i) = m_a_des(m_activeAxes(i));
  }
  v_err_prev = m_v_error;
  return m_constraint;
}

}  // namespace tasks
}  // namespace tsid
