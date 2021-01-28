#ifndef ESVO_CORE_EVENTMATCHER_H
#define ESVO_CORE_EVENTMATCHER_H

#include <vector>
#include <deque>
#include <esvo_core/tools/utils.h>
#include <esvo_core/container/CameraSystem.h>
#include <esvo_core/container/EventMatchPair.h>

namespace esvo_core
{
using namespace container;
using namespace tools;
namespace core
{
struct EventSlice
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  EventSlice(double SLICE_THICKNESS = 2 * 1e-3)
    :SLICE_THICKNESS_(SLICE_THICKNESS){}

  size_t numEvents_;
  double SLICE_THICKNESS_;
  ros::Time t_median_;// used for Transformation iterpolation
  tools::Transformation transf_;
  std::vector<dvs_msgs::Event*>::iterator it_begin_, it_end_;
};

class EventMatcher
{
  struct Job
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    size_t i_thread_;
    std::vector<dvs_msgs::Event*>::iterator vEventPtr_it_begin_;
    std::vector<EventSlice>* pvEventSlice_;
    std::vector<size_t>* pvIndexEventSlice_;
    std::shared_ptr< std::vector<EventMatchPair> > pvEventMatchPair_;
  };
  public:
  EventMatcher(
    CameraSystem::Ptr camSysPtr,
    size_t numThread = 1,
    double Time_THRESHOLD = 10e-5,// 100us
    double EPIPOLAR_THRESHOLD = 0.5,
    double TS_NCC_THRESHOLD = 0.1,
    size_t patch_size_X = 25,
    size_t patch_size_Y = 5,
    size_t patch_intensity_threshold = 125,
    double patch_valid_ratio = 0.1);
  virtual ~EventMatcher();

  void resetParameters(
    double Time_THRESHOLD,
    double EPIPOLAR_THRESHOLD,
    double TS_NCC_THRESHOLD,
    size_t patch_size_X,
    size_t patch_size_Y,
    size_t patch_intensity_threshold,
    double patch_valid_ratio);

  void createMatchProblem(
    StampedTimeSurfaceObs * pTS_obs,
    std::vector<EventSlice>* vEventSlice_ptr,
    std::vector<dvs_msgs::Event*>* vEventPtr_cand);

  bool match_an_event(
    dvs_msgs::Event* ev_ptr,
    Transformation &Trans_world_rv,
    EventMatchPair &emPair);

  void match_all_SingleThread(std::vector<EventMatchPair> &vEMP);
  void match_all_HyperThread(std::vector<EventMatchPair> &vEMP);
  void match(EventMatcher::Job& job);

  double zncc_cost(Eigen::MatrixXd &patch_left, Eigen::MatrixXd &patch_right);

  bool warping2(
    const Eigen::Vector2d &x,
    double invDepth,
    const Eigen::Matrix<double, 3, 4> &T_left_rv,
    Eigen::Vector2d &x1_s,
    Eigen::Vector2d &x2_s);

  bool patchInterpolation2(
    const Eigen::MatrixXd &img,
    const Eigen::Vector2d &location,
    Eigen::MatrixXd &patch);
  private:
  container::CameraSystem::Ptr camSysPtr_;
  std::vector<EventQueue ::iterator> vEvents_left_itr_, vEvents_right_itr_;
  size_t NUM_THREAD_;
  double Time_THRESHOLD_, EPIPOLAR_THRESHOLD_, TS_NCC_THRESHOLD_;
  size_t patch_size_X_, patch_size_Y_;
  size_t patch_intensity_threshold_;
  double patch_valid_ratio_;

  StampedTimeSurfaceObs * pTS_obs_;
  std::vector<EventSlice>* pvEventSlice_;
  std::vector<dvs_msgs::Event*>* pvCandEventPtr_;
};
}
}
#endif //ESVO_CORE_EVENTMATCHER_H
