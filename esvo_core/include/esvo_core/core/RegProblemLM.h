#ifndef ESVO_CORE_CORE_REGPROBLEMLM_H
#define ESVO_CORE_CORE_REGPROBLEMLM_H

#include <esvo_core/container/CameraSystem.h>
#include <esvo_core/container/ResidualItem.h>
#include <esvo_core/optimization/OptimizationFunctor.h>
#include <esvo_core/container/TimeSurfaceObservation.h>
#include <esvo_core/tools/utils.h>

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

namespace esvo_core
{
using namespace container;
using namespace tools;
namespace core
{
struct RegProblemConfig
{
  using Ptr = std::shared_ptr<RegProblemConfig>;
  RegProblemConfig(
    size_t patchSize_X,
    size_t patchSize_Y,
    size_t kernelSize,
    const std::string &LSnorm,
    double huber_threshold,
    double invDepth_min_range = 0.2,
    double invDepth_max_range = 2.0,
    const size_t MIN_NUM_EVENTS = 1000,
    const size_t MAX_REGISTRATION_POINTS = 500,
    const size_t BATCH_SIZE = 200,
    const size_t MAX_ITERATION = 10):
    patchSize_X_(patchSize_X),
    patchSize_Y_(patchSize_Y),
    kernelSize_(kernelSize),
    LSnorm_(LSnorm),
    huber_threshold_(huber_threshold),
    invDepth_min_range_(invDepth_min_range),
    invDepth_max_range_(invDepth_max_range),
    MIN_NUM_EVENTS_(MIN_NUM_EVENTS),
    MAX_REGISTRATION_POINTS_(MAX_REGISTRATION_POINTS),
    BATCH_SIZE_(BATCH_SIZE),
    MAX_ITERATION_(MAX_ITERATION)
  {}

  size_t patchSize_X_, patchSize_Y_;
  size_t kernelSize_;
  std::string LSnorm_;
  double huber_threshold_;
  double invDepth_min_range_;
  double invDepth_max_range_;
  size_t MIN_NUM_EVENTS_;
  size_t MAX_REGISTRATION_POINTS_;
  size_t BATCH_SIZE_;
  size_t MAX_ITERATION_;
};

struct RefFrame
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ros::Time t_;
  std::vector<pcl::PointXYZ*> vPointXYZPtr_;
  Transformation tr_;
};

struct CurFrame
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ros::Time t_;
  TimeSurfaceObservation* pTsObs_;
  Transformation tr_;
  Transformation tr_old_;
  size_t numEventsSinceLastObs_;
};

struct RegProblemLM : public optimization::OptimizationFunctor<double>
{
  struct Job
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    ResidualItems* pvRI_;
    const TimeSurfaceObservation* pTsObs_;
    const Eigen::Matrix4d* T_left_ref_;
    size_t i_thread_;
  };

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  RegProblemLM(
    const CameraSystem::Ptr& camSysPtr,
    const RegProblemConfig::Ptr& rpConfig_ptr,
    size_t numThread = 1);
  void setProblem(RefFrame* ref, CurFrame* cur, bool bComputeGrad = false);
  void setStochasticSampling(size_t offset, size_t N);

  void getWarpingTransformation(
    Eigen::Matrix4d& warpingTransf,
    const Eigen::Matrix<double, 6, 1>& x) const;
  void addMotionUpdate( const Eigen::Matrix<double, 6, 1>& dx );
  void setPose();
  Eigen::Matrix4d getPose();

  // optimization
  int operator()( const Eigen::Matrix<double,6,1>& x, Eigen::VectorXd& fvec ) const;
  void thread( Job& job ) const;
  int df( const Eigen::Matrix<double,6,1>& x, Eigen::MatrixXd& fjac ) const;
  void computeJ_G(const Eigen::Matrix<double,6,1>&x, Eigen::Matrix<double,12,6>& J_G);

  // utils
  bool isValidPatch(
    Eigen::Vector2d& patchCentreCoord,
    Eigen::MatrixXi& mask,
    size_t wx,
    size_t wy) const;

  bool reprojection(
    const Eigen::Vector3d& p,
    const Eigen::Matrix4d& warpingTransf,
    Eigen::Vector2d &x1_s) const;

  bool patchInterpolation(
    const Eigen::MatrixXd &img,
    const Eigen::Vector2d &location,
    Eigen::MatrixXd &patch,
    bool debug = false) const;
  //
  CameraSystem::Ptr camSysPtr_;
  RegProblemConfig::Ptr rpConfigPtr_;
  size_t patchSize_;

  size_t NUM_THREAD_;
  size_t numPoints_;
  size_t numBatches_;

  ResidualItems ResItems_, ResItemsStochSampled_;
  TimeSurfaceObservation* pTsObs_;
  RefFrame* ref_;
  CurFrame* cur_;

  Eigen::Matrix<double,4,4> T_world_left_;// to record the current pose
  Eigen::Matrix<double,4,4> T_world_ref_;// to record the ref pose (local ref map)
  Eigen::Matrix3d R_;//R_ref_cur_;
  Eigen::Vector3d t_;//t_ref_cur

  // Jacobian Constant
  Eigen::Matrix<double,12,6> J_G_0_;
  // debug
  bool bPrint_;
};// struct RegProblemLM
}// namespace core
}// namespace esvo_core

#endif //ESVO_CORE_CORE_REGPROBLEMLM_H