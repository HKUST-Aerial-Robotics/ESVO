#include <esvo_core/core/EventBM.h>
#include <thread>
#include <numeric>
#include <Eigen/Eigenvalues>

esvo_core::core::EventBM::EventBM(
  esvo_core::CameraSystem::Ptr camSysPtr,
  size_t numThread,
  bool bSmoothTS,
  size_t patch_size_X,
  size_t patch_size_Y,
  size_t min_disparity,
  size_t max_disparity,
  size_t step,
  double ZNCC_Threshold,
  bool bUpDownConfiguration):
  camSysPtr_(camSysPtr), sb_(Sobel(3)), NUM_THREAD_(numThread),
  patch_size_X_(patch_size_X), patch_size_Y_(patch_size_Y),
  min_disparity_(min_disparity), max_disparity_(max_disparity), step_(step),
  ZNCC_Threshold_(ZNCC_Threshold),
  bUpDownConfiguration_(bUpDownConfiguration)
{
  ZNCC_MAX_ = 1.0;
  bSmoothTS_ = bSmoothTS;
  /*** For Test ***/
  coarseSearchingFailNum_ = 0;
  fineSearchingFailNum_ = 0;
  infoNoiseRatioLowNum_ = 0;
}

esvo_core::core::EventBM::~EventBM()
{}

void esvo_core::core::EventBM::resetParameters(
  size_t patch_size_X,
  size_t patch_size_Y,
  size_t min_disparity,
  size_t max_disparity,
  size_t step,
  double ZNCC_Threshold,
  bool bUpDownConfiguration)
{
  patch_size_X_  = patch_size_X;
  patch_size_Y_  = patch_size_Y;
  min_disparity_ = min_disparity;
  max_disparity_ = max_disparity;
  step_ = step;
  ZNCC_Threshold_ = ZNCC_Threshold;
  bUpDownConfiguration_ = bUpDownConfiguration;
  /*** For Test ***/
  coarseSearchingFailNum_ = 0;
  fineSearchingFailNum_ = 0;
  infoNoiseRatioLowNum_ = 0;
}

void esvo_core::core::EventBM::createMatchProblem(
  StampedTimeSurfaceObs * pStampedTsObs,
  StampTransformationMap * pSt_map,
  std::vector<dvs_msgs::Event *>* pvEventsPtr)
{
  pStampedTsObs_ = pStampedTsObs;
  pSt_map_ = pSt_map;
  size_t numEvents = pvEventsPtr->size();
  vEventsPtr_.clear();
  vEventsPtr_.reserve(numEvents);
  vEventsPtr_.insert(vEventsPtr_.end(), pvEventsPtr->begin(), pvEventsPtr->end());

  if(bSmoothTS_)
  {
    if(pStampedTsObs_)
      pStampedTsObs_->second.GaussianBlurTS(5);
  }

  vpDisparitySearchBound_.clear();
  vpDisparitySearchBound_.reserve(numEvents);
  for(size_t i = 0; i < vEventsPtr_.size(); i++)
    vpDisparitySearchBound_.push_back(std::make_pair(min_disparity_, max_disparity_));
}

bool esvo_core::core::EventBM::match_an_event(
  dvs_msgs::Event* pEvent,
  std::pair<size_t, size_t>& pDisparityBound,
  esvo_core::core::EventMatchPair& emPair)
{
  size_t lowDisparity = pDisparityBound.first;
  size_t upDisparity  = pDisparityBound.second;
  // rectify and floor the coordinate
  Eigen::Vector2d x_rect = camSysPtr_->cam_left_ptr_->getRectifiedUndistortedCoordinate(pEvent->x, pEvent->y);
  // check if the rectified and undistorted coordinates are outside the image plane. (Added by Yi Zhou on 12 Jan 2021)
  if(x_rect(0) < 0 || x_rect(0) > camSysPtr_->cam_left_ptr_->width_ - 1 ||
     x_rect(1) < 0 || x_rect(1) > camSysPtr_->cam_left_ptr_->height_ - 1)
    return false;
  // This is to avoid depth estimation happenning in the mask area.
  if(camSysPtr_->cam_left_ptr_->UndistortRectify_mask_(x_rect(1), x_rect(0)) <= 125)
    return false;
  Eigen::Vector2i x1(std::floor(x_rect(0)), std::floor(x_rect(1)));
  Eigen::Vector2i x1_left_top;
  if(!isValidPatch(x1, x1_left_top))
    return false;
  // extract the template patch in the left time_surface
  Eigen::MatrixXd patch_src = pStampedTsObs_->second.TS_left_.block(
    x1_left_top(1), x1_left_top(0), patch_size_Y_, patch_size_X_);

  if((patch_src.array() < 1).count() > 0.95 * patch_src.size())
  {
//    LOG(INFO) << "Low info-noise-ratio. @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@";
    infoNoiseRatioLowNum_++;
    return false;
  }

  // LOG(INFO) << "patch_src is extracted";

  // searching along the epipolar line (heading to the left direction)
  double min_cost = ZNCC_MAX_;
  Eigen::Vector2i bestMatch;
  size_t bestDisp;
  Eigen::MatrixXd patch_dst = Eigen::MatrixXd::Zero(patch_size_Y_, patch_size_X_);
  // coarse searching
  if(!epipolarSearching(min_cost, bestMatch, bestDisp, patch_dst,
    lowDisparity, upDisparity, step_,
    x1, patch_src, bUpDownConfiguration_))
  {
//    LOG(INFO) << "Coarse searching fails #################################";
    coarseSearchingFailNum_++;
    return false;
  }
  // fine searching
  size_t fine_searching_start_pos = bestDisp-(step_-1) >= 0 ? bestDisp-(step_-1) : 0;
  if(!epipolarSearching(min_cost, bestMatch, bestDisp, patch_dst,
                    fine_searching_start_pos, bestDisp+(step_-1), 1,
                    x1, patch_src, bUpDownConfiguration_))
  {
    // This indicates the local minima is not surrounded by two neighbors with larger cost,
    // This case happens when the best match locates over/outside the boundary of the Time Surface.
    fineSearchingFailNum_++;
//    LOG(INFO) << "fine searching fails ...............";
    return false;
  }

  // transfer best match to emPair
  if(min_cost <= ZNCC_Threshold_)
  {
    emPair.x_left_raw_ = Eigen::Vector2d((double)pEvent->x, (double)pEvent->y);
    emPair.x_left_ = x_rect;
    emPair.x_right_ = Eigen::Vector2d((double)bestMatch(0), (double)bestMatch(1)) ;
    emPair.t_ = pEvent->ts;
    double disparity;
    if(bUpDownConfiguration_)
      disparity = x1(1) - bestMatch(1);
    else
      disparity = x1(0) - bestMatch(0);
    double depth = camSysPtr_->baseline_ * camSysPtr_->cam_left_ptr_->P_(0,0) / disparity;

    auto st_map_iter = tools::StampTransformationMap_lower_bound(*pSt_map_, emPair.t_);
    if(st_map_iter == pSt_map_->end())
      return false;
    emPair.trans_ = st_map_iter->second;
    emPair.invDepth_ = 1.0 / depth;
    emPair.cost_ = min_cost;
    emPair.disp_ = disparity;
    return true;
  }
  else
  {
//    LOG(INFO) << "BM fails because: " << min_cost << " > " << ZNCC_Threshold_;
    return false;
  }
}

bool esvo_core::core::EventBM::epipolarSearching(
  double& min_cost, Eigen::Vector2i& bestMatch, size_t& bestDisp, Eigen::MatrixXd& patch_dst,
  size_t searching_start_pos, size_t searching_end_pos, size_t searching_step,
  Eigen::Vector2i& x1, Eigen::MatrixXd& patch_src, bool bUpDownConfiguration)
{
  bool bFoundOneMatch = false;
  std::map<size_t, double> mDispCost;

  for(size_t disp = searching_start_pos;disp <= searching_end_pos; disp+=searching_step)
  {
    Eigen::Vector2i x2;
    if(!bUpDownConfiguration)
      x2 << x1(0) - disp, x1(1);
    else
      x2 << x1(0), x1(1) - disp;
    Eigen::Vector2i x2_left_top;
    if(!isValidPatch(x2, x2_left_top))
    {
      mDispCost.emplace(disp, ZNCC_MAX_);
      continue;
    }

    patch_dst = pStampedTsObs_->second.TS_right_.block(
      x2_left_top(1), x2_left_top(0), patch_size_Y_, patch_size_X_);
    double cost = ZNCC_MAX_;
    cost = zncc_cost(patch_src, patch_dst, false);
    mDispCost.emplace(disp, cost);

    if(cost <= min_cost)
    {
      min_cost = cost;
      bestMatch = x2;
      bestDisp = disp;
    }
//    LOG(INFO) << "epipolar searching: " << disp;
  }

  if(searching_step > 1)// coarse
  {
    if(mDispCost.find(bestDisp - searching_step) != mDispCost.end() &&
       mDispCost.find(bestDisp + searching_step) != mDispCost.end())
    {
      if(mDispCost[bestDisp - searching_step] < ZNCC_MAX_ && mDispCost[bestDisp + searching_step] < ZNCC_MAX_ )
        if(min_cost < ZNCC_Threshold_)
          bFoundOneMatch = true;
//      else
//        LOG(INFO) << "coarse searching fails: " << mDispCost[bestDisp - searching_step] << " <-> "
//                  << mDispCost[bestDisp + searching_step];
    }
  }
  else// fine
  {
    if(min_cost < ZNCC_Threshold_)
      bFoundOneMatch = true;
  }
  return bFoundOneMatch;
}

void esvo_core::core::EventBM::match_all_SingleThread(
  std::vector<EventMatchPair> &vEMP)
{
  coarseSearchingFailNum_ = 0;
  fineSearchingFailNum_ = 0;
  infoNoiseRatioLowNum_ = 0;

  vEMP.clear();
  vEMP.reserve(vEventsPtr_.size());
  for(size_t i = 0; i < vEventsPtr_.size(); i++)
  {
    EventMatchPair emp;
    std::pair<size_t, size_t> pDisparityBound = vpDisparitySearchBound_[i];
    if(match_an_event(vEventsPtr_[i], pDisparityBound, emp))
      vEMP.emplace_back(emp);
  }
//  LOG(INFO) << "Total number of events: " << vEventsPtr_.size();
//  LOG(INFO) << "Info-noise ratio low # " << infoNoiseRatioLowNum_;
//  LOG(INFO) << "coarse searching fails # " << coarseSearchingFailNum_;
//  LOG(INFO) << "fine searching fails # " << fineSearchingFailNum_;
//  LOG(INFO) << "match_all_SingleThread: " << vEMP.size();
}

bool esvo_core::core::EventBM::isValidPatch(
  Eigen::Vector2i& x,
  Eigen::Vector2i& left_top)
{
  int wx = (patch_size_X_ - 1) / 2;
  int wy = (patch_size_Y_ - 1) / 2;
  left_top = Eigen::Vector2i(x(0) - wx, x(1) - wy);
  Eigen::Vector2i right_bottom(x(0) + wx, x(1) + wy);
  // NOTE: The patch cannot touch the boundary row/col of the orginal image,
  // since in the nonlinear optimization, the interpolation would access
  // the neighbouring row/col!!!
  if(left_top(0) < 1 || left_top(1) < 1 ||
     right_bottom(0) >= camSysPtr_->cam_left_ptr_->width_ - 1 ||
     right_bottom(1) >= camSysPtr_->cam_left_ptr_->height_ - 1 )
    return false;
  return true;
}

void esvo_core::core::EventBM::match_all_HyperThread(
  vector<EventMatchPair> &vEMP)
{
  std::vector<EventBM::Job> jobs(NUM_THREAD_);
  for(size_t i = 0;i < NUM_THREAD_; i++)
  {
    jobs[i].i_thread_ = i;
    jobs[i].pvEventPtr_ = &vEventsPtr_;
    jobs[i].pvpDisparitySearchBound_ = &vpDisparitySearchBound_;
    jobs[i].pvEventMatchPair_ = std::make_shared<std::vector<EventMatchPair> >();
  }

  std::vector<std::thread> threads;
  threads.reserve(NUM_THREAD_);
  for(size_t i = 0; i< NUM_THREAD_; i++)
    threads.emplace_back(std::bind(&EventBM::match, this, jobs[i]));
  for(auto & thread : threads)
  {
    if(thread.joinable())
      thread.join();
  }
  size_t numPoints = 0;
  for(size_t i = 0;i < NUM_THREAD_;i++)
    numPoints += jobs[i].pvEventMatchPair_->size();
  vEMP.clear();
  vEMP.reserve(numPoints);
  for(size_t i = 0;i < NUM_THREAD_;i++)
    vEMP.insert(vEMP.end(), jobs[i].pvEventMatchPair_->begin(), jobs[i].pvEventMatchPair_->end());
}

void esvo_core::core::EventBM::match(
  EventBM::Job& job)
{
  size_t i_thread = job.i_thread_;
  size_t totalNumEvents = job.pvEventPtr_->size();
  job.pvEventMatchPair_->reserve(totalNumEvents / NUM_THREAD_ + 1);

  auto ev_it = job.pvEventPtr_->begin();
  std::advance(ev_it, i_thread);
  for(size_t i = i_thread; i < totalNumEvents; i+=NUM_THREAD_, std::advance(ev_it, NUM_THREAD_))
  {
    EventMatchPair emp;
    std::pair<size_t, size_t> pDisparityBound = (*job.pvpDisparitySearchBound_)[i];
    if(match_an_event(*ev_it, pDisparityBound, emp))
      job.pvEventMatchPair_->push_back(emp);
  }
}

double esvo_core::core::EventBM::zncc_cost(
  Eigen::MatrixXd &patch_left,
  Eigen::MatrixXd &patch_right,
  bool normalized)
{
  double cost;
  if(!normalized)
  {
    Eigen::MatrixXd patch_left_normalized, patch_right_normalized;
    tools::normalizePatch(patch_left, patch_left_normalized);
    tools::normalizePatch(patch_right, patch_right_normalized);
    cost = 0.5 * (1 - (patch_left_normalized.array() * patch_right_normalized.array()).sum() / (patch_left.rows() * patch_left.cols()));
  }
  else
    cost = 0.5 * (1 - (patch_left.array() * patch_right.array()).sum() / (patch_left.rows() * patch_left.cols()));
  return cost;
}