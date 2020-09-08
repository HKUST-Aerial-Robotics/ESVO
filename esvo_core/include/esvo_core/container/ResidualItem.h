#ifndef ESVO_CORE_CONTAINER_RESIDUALITEM_H
#define ESVO_CORE_CONTAINER_RESIDUALITEM_H

#include <Eigen/Eigen>
#include <vector>
#include <memory>

namespace esvo_core
{
namespace container
{
struct ResidualItem
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d p_;// 3D coordinate in the reference frame
  Eigen::Vector2d p_img_;// 2D coordinate in the image plane

//  double IRLS_weight_;
//  double variance_;
  Eigen::VectorXd residual_;
//  bool bOutlier_;

  ResidualItem();
  ResidualItem(const double x,const double y,const double z);
  void initialize(const double x,const double y,const double z);
};

using ResidualItems = std::vector<ResidualItem, Eigen::aligned_allocator<ResidualItem> >;
}// namespace container
}// namespace esvo_core

#endif //ESVO_CORE_CONTAINER_RESIDUALITEM_H
