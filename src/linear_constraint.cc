/**
 @file    a_linear_constraint.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 26, 2016
 @brief   Brief description
 */

#include <xpp/opt/constraints/linear_constraint.h>

#include <vector>
#include <Eigen/Dense>
#include <Eigen/Sparse>

#include <xpp/bound.h>
#include <xpp/opt/variables/base_motion.h>

namespace xpp {
namespace opt {

LinearEqualityConstraint::LinearEqualityConstraint (
    const OptVarsPtr& opt_vars,
    const MatVec& linear_equation)
{
  linear_equation_ = linear_equation;
//  name_ = name;

  com_motion_ = std::dynamic_pointer_cast<BaseMotion>(opt_vars->GetSet("base_motion"));
  opt_vars_ = opt_vars;

  int num_constraints = linear_equation_.v.rows();
  SetDimensions(opt_vars, num_constraints);

  // zmp_ remove
//  Jacobian& jac = GetJacobianRefWithRespectTo(com_motion_->GetId());
//
//  jac = linear_equation_.M.sparseView();
}

LinearEqualityConstraint::~LinearEqualityConstraint ()
{
}

LinearEqualityConstraint::VectorXd
LinearEqualityConstraint::GetConstraintValues () const
{
  VectorXd x = com_motion_->GetXYSplineCoeffients();
  return linear_equation_.M*x;
}

VecBound
LinearEqualityConstraint::GetBounds () const
{
  VecBound bounds;

  for (int i=0; i<GetNumberOfConstraints(); ++i) {
    Bound bound(-linear_equation_.v[i],-linear_equation_.v[i]);
    bounds.push_back(bound);
  }

  return bounds;
}

LinearEqualityConstraint::Jacobian
LinearEqualityConstraint::GetJacobianWithRespectTo (std::string var_set) const
{
  int n = opt_vars_->GetSet(var_set)->GetOptVarCount();
  Jacobian jac = Jacobian(num_constraints_, n);

  // the constraints are all linear w.r.t. the decision variables.
  // careful, .sparseView is only valid when the Jacobian is constant, e.g.
  if (var_set == com_motion_->GetId())
    jac = linear_equation_.M.sparseView();

  return jac;
}

} /* namespace opt */
} /* namespace xpp */

