#ifndef ESVO_CORE_CORE_DEPTHFUSION_H
#define ESVO_CORE_CORE_DEPTHFUSION_H

#include <esvo_core/container/CameraSystem.h>
#include <esvo_core/core/DepthProblem.h>
#include <esvo_core/container/DepthPoint.h>
#include <esvo_core/container/DepthMap.h>
#include <esvo_core/tools/utils.h>
#include <mutex>

namespace esvo_core
{
using namespace container;
namespace core
{
class DepthFusion
{
  public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  DepthFusion(
    CameraSystem::Ptr &camSysPtr,
    std::shared_ptr<DepthProblemConfig> & dpConfigPtr);

  virtual ~DepthFusion();

  bool propagate_one_point(
    DepthPoint &dp_prior,
    DepthPoint &dp_prop,
    Eigen::Matrix<double, 4, 4> &T_prop_prior);

  int fusion(
    DepthPoint &dp_prop,
    DepthMap::Ptr &dm,
    int fusion_radius = 0);

  int update(
    std::vector<DepthPoint> &dp_obs,
    DepthFrame::Ptr &df,
    int fusion_radius = 0);

  bool boundaryCheck(
    double xcoor,
    double ycoor,
    size_t width,
    size_t height);

  bool chiSquareTest(
    double invD1, double invD2,
    double var1, double var2);

  bool studentTCompatibleTest(
    double invD1, double invD2,
    double var1, double var2);

  // Used by GTL and SGM for comparison, and also used in the initialization.
  void naive_propagation(
    std::vector<DepthPoint> &dp_obs,
    DepthFrame::Ptr &df);

  private:
  CameraSystem::Ptr camSysPtr_;
  std::shared_ptr<DepthProblemConfig> dpConfigPtr_;
};
}
}

#endif //ESVO_CORE_CORE_DEPTHFUSION_H