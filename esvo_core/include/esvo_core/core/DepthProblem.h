#ifndef ESVO_CORE_CORE_DEPTHPROBLEM_H
#define ESVO_CORE_CORE_DEPTHPROBLEM_H

#include <esvo_core/tools/utils.h>
#include <esvo_core/container/CameraSystem.h>
#include <esvo_core/container/TimeSurfaceObservation.h>
#include <esvo_core/optimization/OptimizationFunctor.h>

namespace esvo_core
{
using namespace container;
using namespace tools;
namespace core
{
struct DepthProblemConfig
{
  using Ptr = std::shared_ptr<DepthProblemConfig>;
  DepthProblemConfig(
    size_t patchSize_X,
    size_t patchSize_Y,
    const std::string &LSnorm,
    double td_nu,
    double td_scale,
    size_t MAX_ITERATION,
    size_t RegularizationRadius,
    size_t RegularizationMinNeighbours,
    size_t RegularizationMinCloseNeighbours):
    patchSize_X_(patchSize_X),
    patchSize_Y_(patchSize_Y),
    LSnorm_(LSnorm),
    td_nu_(td_nu),
    td_scale_(td_scale),
    td_scaleSquared_(pow(td_scale,2)),
    td_stdvar_(sqrt(td_nu / (td_nu - 2) * td_scaleSquared_)),
    MAX_ITERATION_(MAX_ITERATION),
    RegularizationRadius_(RegularizationRadius),
    RegularizationMinNeighbours_(RegularizationMinNeighbours),
    RegularizationMinCloseNeighbours_(RegularizationMinCloseNeighbours)
    {}

  size_t patchSize_X_, patchSize_Y_;
  std::string LSnorm_;
  double td_nu_;
  double td_scale_;
  double td_scaleSquared_;// td_scale_^2
  double td_stdvar_;// sigma
  size_t MAX_ITERATION_;
  size_t RegularizationRadius_;
  size_t RegularizationMinNeighbours_;
  size_t RegularizationMinCloseNeighbours_;
};

struct DepthProblem : public optimization::OptimizationFunctor<double>
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  DepthProblem(
    const DepthProblemConfig::Ptr & dpConfig_ptr,
    const CameraSystem::Ptr & camSysPtr );

  void setProblem(
    Eigen::Vector2d & coor,
    Eigen::Matrix<double, 4, 4> & T_world_virtual,
    StampedTimeSurfaceObs* pStampedTsObs);

  // function that is inherited from optimization::OptimizationFunctor
  int operator()( const Eigen::VectorXd &x, Eigen::VectorXd & fvec ) const;

  // utils
  bool warping(
    const Eigen::Vector2d &x,
    double d,
    const Eigen::Matrix<double, 3, 4> &T_left_virtual,
    Eigen::Vector2d &x1_s,
    Eigen::Vector2d &x2_s) const;

  bool patchInterpolation(
    const Eigen::MatrixXd &img,
    const Eigen::Vector2d &location,
    Eigen::MatrixXd &patch,
    bool debug = false) const;

  // variables
  CameraSystem::Ptr camSysPtr_;
  DepthProblemConfig::Ptr dpConfigPtr_;
  Eigen::Vector2d coordinate_;
  Eigen::Matrix<double,4,4> T_world_virtual_;
  std::vector<Eigen::Matrix<double, 3, 4>,
    Eigen::aligned_allocator<Eigen::Matrix<double, 3, 4> > > vT_left_virtual_;
  StampedTimeSurfaceObs* pStampedTsObs_;
};
}
}


#endif //ESVO_CORE_CORE_DEPTHPROBLEM_H
