#ifndef ESVO_CORE_CONTAINER_DEPTHPOINT_H
#define ESVO_CORE_CONTAINER_DEPTHPOINT_H

#include <memory>
#include <stdlib.h>
#include <Eigen/Eigen>

namespace esvo_core
{
namespace container
{
class DepthPoint
{
  public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef std::shared_ptr<DepthPoint> Ptr;

  DepthPoint();

  DepthPoint(size_t row, size_t col);
  virtual ~DepthPoint();

  size_t row() const;
  size_t col() const;

  void update_x(const Eigen::Vector2d &x);
  const Eigen::Vector2d &x() const;

  double &invDepth();
  const double &invDepth() const;

  double &scaleSquared();
  const double &scaleSquared() const;

  double &nu();
  const double &nu() const;

  double &variance();
  const double &variance() const;

  double &residual();
  const double &residual() const;

  size_t &age();
  const size_t &age() const;

  void boundVariance();

  void update(double invDepth, double variance);// Gaussian distribution
  void update_studentT(double invDepth, double scale2, double variance, double nu); // T distribution

  void update_p_cam(const Eigen::Vector3d &p);
  const Eigen::Vector3d &p_cam() const;

  void updatePose(Eigen::Matrix<double, 4, 4> &T_world_cam);// used in the fusion of each newly estimate.
  // Therefore, it is not necessary to call updatePose for those created in the fusion. Because those share
  // the pose of the fused depthFrame.

  const Eigen::Matrix<double, 4, 4> &T_world_cam() const;

  bool valid() const;
  bool valid(double var_threshold,
             double age_threshold,
             double invDepth_max,
             double invDepth_min) const;

  //copy an element without the location
  void copy(const DepthPoint &copy);

  private:
  //coordinates in the image
  size_t row_;
  size_t col_;
  Eigen::Vector2d x_;

  //inverse depth parameters
  double invDepth_;
  double scaleSquared_;// squared scale
  double nu_;
  double variance_;
  double residual_;

  // count the number of fusion has been applied on a depth point
  size_t age_;

  //3D point (updated in reference frame before tracking)
  Eigen::Vector3d p_cam_;
  Eigen::Matrix<double, 4, 4> T_world_cam_;
};
}
}

#endif //ESVO_CORE_CONTAINER_DEPTHPOINT_H