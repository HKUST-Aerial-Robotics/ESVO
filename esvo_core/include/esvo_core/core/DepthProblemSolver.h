#ifndef ESVO_CORE_CORE_DEPTHPROBLEMSOLVER2_H
#define ESVO_CORE_CORE_DEPTHPROBLEMSOLVER2_H

#include <memory>
#include <esvo_core/core/DepthProblem.h>
#include <esvo_core/optimization/OptimizationFunctor.h>
#include <unsupported/Eigen/NonLinearOptimization>
#include <unsupported/Eigen/NumericalDiff>
#include <esvo_core/container/EventMatchPair.h>

namespace esvo_core
{
namespace core
{
enum DepthProblemType
{
  ANALYTICAL,
  NUMERICAL
};
class DepthProblemSolver
{
  struct Job
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    size_t i_thread_;
    std::vector<EventMatchPair> *pvEMP_;
    StampedTimeSurfaceObs* pStamped_TS_obs_;
    std::shared_ptr<DepthProblem> dProblemPtr_;
    std::shared_ptr< Eigen::NumericalDiff<DepthProblem> > numDiff_dProblemPtr_;
    std::shared_ptr< std::vector<DepthPoint> > vdpPtr_;
  };

  public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  DepthProblemSolver(
    CameraSystem::Ptr & camSysPtr,
    std::shared_ptr<DepthProblemConfig> & dpConfigPtr,
    DepthProblemType dpType = NUMERICAL, // we only provide the numerical (jacobian) solver here, because the analytical one will not show remarkable efficiency.
    size_t numThread = 1);

  virtual ~DepthProblemSolver();

  void solve(
    std::vector<EventMatchPair>* pvEMP,
    StampedTimeSurfaceObs* pStampedTsObs,
    std::vector<DepthPoint> &vdp );

  void solve_multiple_problems(Job &job);

  bool solve_single_problem_numerical(
    double d_init,
    std::shared_ptr< Eigen::NumericalDiff<DepthProblem> > & dProblemPtr,
    double* result);

  void pointCulling(
    std::vector<DepthPoint> &vdp,
    double std_variance_threshold,
    double cost_threshold,
    double invDepth_min_range,
    double invDepth_max_range);

  DepthProblemType getProblemType();

  private:
  CameraSystem::Ptr & camSysPtr_;
  std::shared_ptr<DepthProblemConfig> dpConfigPtr_;
  size_t NUM_THREAD_;
  DepthProblemType dpType_;
};
}
}
#endif //ESVO_CORE_CORE_DEPTHPROBLEMSOLVER2_H