/**
@file    spliner_3d.cc
@author  Alexander Winkler (winklera@ethz.ch)
@date    Oct 21, 2014
@brief   Creates 3 dimensional spline from start to end with duration T
 */

#include <cassert>

namespace xpp {
namespace opt {

template<typename PolynomialType, typename PointType>
PolynomialXd<PolynomialType, PointType>::PolynomialXd ()
{
}

template<typename PolynomialType, typename PointType>
PolynomialXd<PolynomialType, PointType>::PolynomialXd (double duration)
{
  SetBoundary(duration, PointT(), PointT());
}

template<typename PolynomialType, typename PointType>
PolynomialXd<PolynomialType, PointType>::~PolynomialXd ()
{
}

template<typename PolynomialType, typename PointType>
const double
PolynomialXd<PolynomialType, PointType>::GetDuration () const
{
  // all polynomials have same duration, so just return duration of X
  return polynomials_.at(X).GetDuration();
}

template<typename PolynomialType, typename PointType>
typename PolynomialXd<PolynomialType, PointType>::Vector
PolynomialXd<PolynomialType, PointType>::GetState (MotionDerivative pos_vel_acc_jerk,
                                                   double t) const
{
  PointT p = GetPoint(t);
  return p.GetByIndex(pos_vel_acc_jerk);
}

template<typename PolynomialType, typename PointType>
const double
PolynomialXd<PolynomialType, PointType>::GetCoefficient (int dim, PolyCoeff coeff) const
{
  return polynomials_.at(dim).GetCoefficient(coeff);
}

template<typename PolynomialType, typename PointType>
void
PolynomialXd<PolynomialType, PointType>::SetCoefficients (int dim, PolyCoeff coeff,
                                                          double value)
{
  polynomials_.at(dim).SetCoefficient(coeff,value);
}

template<typename PolynomialType, typename PointType>
void PolynomialXd<PolynomialType, PointType>::SetBoundary(double T,
                                                          const PointT& start,
                                                          const PointT& end)
{
  for (int dim=X; dim<kNumDim; ++dim)
    polynomials_.at(dim).SetBoundary(T, start.Get1d(dim), end.Get1d(dim));
}

template<typename PolynomialType, typename PointType>
PointType
PolynomialXd<PolynomialType, PointType>::GetPoint(const double dt) const
{
  PointT p;
  for (int dim=X; dim<kNumDim; ++dim)
    p.SetDimension(polynomials_.at(dim).GetPoint(dt), dim);

  return p;
}

template<typename PolynomialType, typename PointType>
const PolynomialType
PolynomialXd<PolynomialType, PointType>::GetDim (int dim) const
{
  return polynomials_.at(dim);
}

///////////////////////////////////////////////////////////////////////////////

template<typename TPolyXd>
double
ComPolynomialHelpers<TPolyXd>::GetTotalTime(
    const VecPolynomials& splines)
{
  double T = 0.0;
  for (const auto& s: splines)
    T += s.GetDuration();
  return T;
}

template<typename TPolyXd>
double
ComPolynomialHelpers<TPolyXd>::GetLocalTime(
    double t_global, const VecPolynomials& splines)
{
  int id_spline = GetPolynomialID(t_global,splines);

  double t_local = t_global;
  for (int id=0; id<id_spline; id++) {
    t_local -= splines.at(id).GetDuration();
  }

  return t_local;//-eps_; // just to never get value greater than true duration due to rounding errors
}

template<typename TPolyXd>
typename ComPolynomialHelpers<TPolyXd>::PointType
ComPolynomialHelpers<TPolyXd>::GetCOM(
    double t_global, const VecPolynomials& splines)
{
  int idx        = GetPolynomialID(t_global,splines);
  double t_local = GetLocalTime(t_global, splines);

  return GetCOGxyAtPolynomial(idx, t_local, splines);
}

template<typename TPolyXd>
typename ComPolynomialHelpers<TPolyXd>::PointType
ComPolynomialHelpers<TPolyXd>::GetCOGxyAtPolynomial (
    int idx, double t_local, const VecPolynomials& splines)
{
  StateLin2d cog_xy;
  cog_xy.p = splines[idx].GetState(kPos, t_local);
  cog_xy.v = splines[idx].GetState(kVel, t_local);
  cog_xy.a = splines[idx].GetState(kAcc, t_local);
  cog_xy.j = splines[idx].GetState(kJerk, t_local);

  return cog_xy;
}

template<typename TPolyXd>
int
ComPolynomialHelpers<TPolyXd>::GetPolynomialID(
    double t_global, const VecPolynomials& splines)
{
  double eps = 1e-10; // double imprecision
  assert(t_global<=GetTotalTime(splines)+eps); // machine precision

   double t = 0;
   int i=0;
   for (const auto& s: splines) {
     t += s.GetDuration();

     if (t >= t_global-eps) // at junctions, returns previous spline (=)
       return i;

     i++;
   }
   assert(false); // this should never be reached
}

} // namespace opt
} // namespace xpp
