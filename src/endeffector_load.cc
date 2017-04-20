/**
 @file    endeffector_load.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Mar 16, 2017
 @brief   Brief description
 */

#include <xpp/opt/variables/endeffector_load.h>

#include <cmath>
#include <vector>

#include <xpp/bound.h>

namespace xpp {
namespace opt {

EndeffectorLoad::EndeffectorLoad (int num_ee, double dt, double T)
    : OptimizationVariables("endeffector_load")
{
  dt_ = dt;
  T_ = T;
  n_ee_ = num_ee;
  int idx_segment = GetSegment(T);
  num_segments_ = idx_segment + 1;
  int num_parameters = n_ee_ * num_segments_;
  lambdas_ = VectorXd::Ones(num_parameters);
}

EndeffectorLoad::~EndeffectorLoad ()
{
}

void
EndeffectorLoad::SetVariables (const VectorXd& x)
{
  lambdas_ = x;
}

EndeffectorLoad::VectorXd
EndeffectorLoad::GetVariables () const
{
  return lambdas_;
}

int
EndeffectorLoad::GetSegment (double t) const
{
  return floor(t/dt_);
}

EndeffectorLoad::LoadParams
EndeffectorLoad::GetLoadValues (double t) const
{
  int k = GetSegment(t);
  return GetLoadValuesIdx(k);
}

EndeffectorLoad::LoadParams
EndeffectorLoad::GetLoadValuesIdx (int k_curr) const
{
  LoadParams load(n_ee_);
  for (auto ee : load.GetEEsOrdered())
    load.At(ee) = lambdas_(IndexDiscrete(k_curr,ee));

  return load;
}

int
EndeffectorLoad::Index (double t, EndeffectorID ee) const
{
  int k = GetSegment(t);
  return IndexDiscrete(k, ee);
}

int
EndeffectorLoad::IndexDiscrete (int k_curr, EndeffectorID ee) const
{
  return n_ee_*k_curr + ee;
}

int
EndeffectorLoad::GetNumberOfSegments () const
{
  return num_segments_;
}

double
EndeffectorLoad::GetTimeCenterSegment (int segment_id) const
{
  double t_start = segment_id*dt_;

  if (segment_id == num_segments_-1) // last segment might have different length
    return (t_start + T_)/2.;
  else
    return t_start + dt_/2.;
}


} /* namespace opt */
} /* namespace xpp */

