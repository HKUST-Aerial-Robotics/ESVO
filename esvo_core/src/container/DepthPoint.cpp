#include <esvo_core/container/DepthPoint.h>

namespace esvo_core
{
namespace container
{
DepthPoint::DepthPoint()
{
  row_ = 0;
  col_ = 0;
  x_ = Eigen::Vector2d(col_ + 0.5, row_ + 0.5);

  invDepth_ = -1.0;
  variance_ = 0.0;
  residual_ = 0.0;

  age_ = 0;
//  outlierProbability_ = 0.0;
}

DepthPoint::DepthPoint(
  size_t row, size_t col)
{
  row_ = row;
  col_ = col;
  x_ = Eigen::Vector2d(col_ + 0.5, row_ + 0.5);

  invDepth_ = -1.0;
  variance_ = 0.0;
  residual_ = 0.0;

  age_ = 0;
//  outlierProbability_ = 0.0;
}

DepthPoint::~DepthPoint()
{

}

size_t
DepthPoint::row() const
{
  return row_;
}

size_t
DepthPoint::col() const
{
  return col_;
}

const Eigen::Vector2d &
DepthPoint::x() const
{
  return x_;
}

void
DepthPoint::update_x(const Eigen::Vector2d &x)
{
  x_ = x;
}

double &
DepthPoint::invDepth()
{
  return invDepth_;
}

const double &
DepthPoint::invDepth() const
{
  return invDepth_;
}

double &
DepthPoint::scaleSquared()
{
  return scaleSquared_;
}

const double &
DepthPoint::scaleSquared() const
{
  return scaleSquared_;
}

double &
DepthPoint::nu()
{
  return nu_;
}

const double &
DepthPoint::nu() const
{
  return nu_;
}

double &
DepthPoint::variance()
{
  return variance_;
}

const double &
DepthPoint::variance() const
{
  return variance_;
}

double &
DepthPoint::residual()
{
  return residual_;
}

const double &
DepthPoint::residual() const
{
  return residual_;
}

size_t &
DepthPoint::age()
{
  return age_;
}

const size_t &
DepthPoint::age() const
{
  return age_;
}

void
DepthPoint::boundVariance()
{
  double eps = 1e-6;
  if (variance_ < eps)
    variance_ = eps;
}

void
DepthPoint::update(
  double invDepth, double variance)
{
  if (invDepth_ > -1e-6)
  {
    //do an actual update
    double temp = invDepth_;
    invDepth_ = (variance_ * invDepth + variance * temp) / (variance_ + variance);
    temp = variance_;
    variance_ = (temp * variance) / (temp + variance);
  }
  else
  {
    //this is a new point, so simply take the first measurement
    invDepth_ = invDepth;
    variance_ = variance;
  }
  boundVariance();
}

void
DepthPoint::update_studentT(double invDepth, double scale2, double variance, double nu)
{
  if(invDepth_ > -1e-6)
  {
    double nu_update = std::min(nu, nu_);
    double invDepth_update = (scale2*invDepth_ + scaleSquared_ * invDepth) / (scaleSquared_ + scale2);
    double scale2_update = (nu_update + pow(invDepth_ - invDepth,2) / (scaleSquared_ + scale2)) / (nu_update + 1) * (scaleSquared_ * scale2) / (scaleSquared_ + scale2);

    invDepth_ = invDepth_update;
    scaleSquared_ = scale2_update;
    nu_ = nu_update + 1;
    variance_ = nu_ / (nu_ - 2) * scaleSquared_;
    age_++;
  }
  else
  {
    invDepth_ = invDepth;
    scaleSquared_ = scale2;
    variance_ = variance;
    nu_ = nu;
  }
}

void
DepthPoint::update_p_cam(const Eigen::Vector3d &p)
{
  p_cam_ = p;
}

const Eigen::Vector3d &
DepthPoint::p_cam() const
{
  return p_cam_;
}

void
DepthPoint::updatePose(Eigen::Matrix<double, 4, 4> &T_world_cam)
{
  T_world_cam_ = T_world_cam;
}

const Eigen::Matrix<double, 4, 4> &
DepthPoint::T_world_cam() const
{
  return T_world_cam_;
}

bool
DepthPoint::valid() const
{
  return invDepth_ > -1e-6;
}

bool
DepthPoint::valid(double var_threshold,
                  double age_threshold,
                  double invDepth_max,
                  double invDepth_min) const
{
  return invDepth_ > -1e-6 &&
         age_ >= age_threshold &&
         variance_ <= var_threshold &&
         invDepth_ <= invDepth_max &&
         invDepth_ >= invDepth_min;
}

void
DepthPoint::copy(const DepthPoint &copy)
{
  invDepth_ = copy.invDepth_;
  variance_ = copy.variance_;
  scaleSquared_ = copy.scaleSquared_;
  nu_ = copy.nu_;
  x_ = copy.x_;
  p_cam_ = copy.p_cam_;
  T_world_cam_ = copy.T_world_cam_;
  residual_ = copy.residual_;
  age_ = copy.age_;
}

}

}