#ifndef ESVO_CORE_CORE_EVENTBM_H
#define ESVO_CORE_CORE_EVENTBM_H
#include <Eigen/Eigen>
#include <esvo_core/tools/utils.h>
#include <esvo_core/tools/sobel.h>
#include <esvo_core/container/CameraSystem.h>
#include <esvo_core/container/DepthMap.h>
#include <esvo_core/container/EventMatchPair.h>

namespace esvo_core
{
using namespace tools;
using namespace container;
namespace core
{
class EventBM
{
  struct Job
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    size_t i_thread_;
    std::vector<dvs_msgs::Event*>* pvEventPtr_;
    std::vector<std::pair<size_t, size_t> >* pvpDisparitySearchBound_;
    std::shared_ptr<std::vector<EventMatchPair> > pvEventMatchPair_;
  };
public:
  EventBM(
    CameraSystem::Ptr camSysPtr,
    size_t numThread = 1,
    bool bSmoothTS = false,
    size_t patch_size_X = 25,
    size_t patch_size_Y = 25,
    size_t min_disparity = 1,
    size_t max_disparity = 40,
    size_t step = 1,
    double ZNCC_Threshold = 0.1,
    bool bUpDownConfiguration = false);// bUpDownConfiguration determines the epipolar searching direction (UpDown or LeftRight).
  virtual ~EventBM();

  void resetParameters(
    size_t patch_size_X,
    size_t patch_size_Y,
    size_t min_disparity,
    size_t max_disparity,
    size_t step,
    double ZNCC_Threshold,
    bool bDonwUpConfiguration);

  void createMatchProblem(
    StampedTimeSurfaceObs * pStampedTsObs,
    StampTransformationMap * pSt_map,
    std::vector<dvs_msgs::Event *>* pvEventsPtr);

  bool match_an_event(
    dvs_msgs::Event *pEvent,
    std::pair<size_t, size_t>& pDisparityBound,
    EventMatchPair &emPair);

  void match_all_SingleThread(std::vector<EventMatchPair> &vEMP);
  void match_all_HyperThread(std::vector<EventMatchPair> &vEMP);

  static double zncc_cost(Eigen::MatrixXd &patch_left, Eigen::MatrixXd &patch_right, bool normalized = false);

private:
  void match(EventBM::Job &job);
  bool epipolarSearching(double& min_cost, Eigen::Vector2i& bestMatch, size_t& bestDisp, Eigen::MatrixXd& patch_dst,
                         size_t searching_start_pos, size_t searching_end_pos, size_t searching_step,
                         Eigen::Vector2i& x1, Eigen::MatrixXd& patch_src, bool bDownUpConfiguration = false);
  bool isValidPatch(Eigen::Vector2i& x, Eigen::Vector2i& left_top);
private:
  CameraSystem::Ptr camSysPtr_;
  StampedTimeSurfaceObs* pStampedTsObs_;
  StampTransformationMap * pSt_map_;
  std::vector<dvs_msgs::Event*> vEventsPtr_;
  std::vector<std::pair<size_t, size_t> > vpDisparitySearchBound_;
  Sobel sb_;

  size_t NUM_THREAD_;
  bool bSmoothTS_;
  size_t patch_size_X_;
  size_t patch_size_Y_;
  size_t min_disparity_;
  size_t max_disparity_;
  size_t step_;
  double ZNCC_Threshold_;
  double ZNCC_MAX_;
  bool bUpDownConfiguration_;

  size_t coarseSearchingFailNum_, fineSearchingFailNum_, infoNoiseRatioLowNum_;
};
}// core
}// esvo_core

#endif //ESVO_CORE_CORE_EVENTBM_H
