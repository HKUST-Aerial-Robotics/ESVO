// This cpp file implements the following paper for comparison.
// [26] S.-H. Ieng, J. Carneiro, M. Osswald, and R. Benosman, “Neuromorphic event-based generalized time-based stereovision,” Front. Neurosci.,
// vol. 12, p. 442, 2018.
// Note that the photometric consistency check is not applicable because corresponding brightness information is not available for DVS sensors.
#include <esvo_core/core/EventMatcher.h>
#include <esvo_core/tools/TicToc.h>
#include <thread>

esvo_core::core::EventMatcher::EventMatcher(
  CameraSystem::Ptr camSysPtr,
  size_t numThread,
  double Time_THRESHOLD,
  double EPIPOLAR_THRESHOLD,
  double TS_NCC_THRESHOLD,
  size_t patch_size_X,
  size_t patch_size_Y,
  size_t patch_intensity_threshold,
  double patch_valid_ratio):
  camSysPtr_(camSysPtr),
  NUM_THREAD_(numThread),
  Time_THRESHOLD_(Time_THRESHOLD),
  EPIPOLAR_THRESHOLD_(EPIPOLAR_THRESHOLD),
  TS_NCC_THRESHOLD_(TS_NCC_THRESHOLD),
  patch_size_X_(patch_size_X),
  patch_size_Y_(patch_size_Y),
  patch_intensity_threshold_(patch_intensity_threshold),
  patch_valid_ratio_(patch_valid_ratio)
{}

esvo_core::core::EventMatcher::~EventMatcher(){}

void esvo_core::core::EventMatcher::resetParameters(
  double Time_THRESHOLD,
  double EPIPOLAR_THRESHOLD,
  double TS_NCC_THRESHOLD,
  size_t patch_size_X,
  size_t patch_size_Y,
  size_t patch_intensity_threshold,
  double patch_valid_ratio)
{
  Time_THRESHOLD_ = Time_THRESHOLD;
  EPIPOLAR_THRESHOLD_ = EPIPOLAR_THRESHOLD;
  TS_NCC_THRESHOLD_ = TS_NCC_THRESHOLD;
  patch_size_X_ = patch_size_X;
  patch_size_Y_ = patch_size_Y;
  patch_intensity_threshold_ = patch_intensity_threshold;
  patch_valid_ratio_ = patch_valid_ratio;
}

void esvo_core::core::EventMatcher::createMatchProblem(
  StampedTimeSurfaceObs* pTS_obs,
  std::vector<EventSlice>* vEventSlice_ptr,
  std::vector<dvs_msgs::Event*>* vEventPtr_cand)
{
  pTS_obs_ = pTS_obs;
  pvEventSlice_ = vEventSlice_ptr;
  pvCandEventPtr_ = vEventPtr_cand;
}

bool esvo_core::core::EventMatcher::match_an_event(
  dvs_msgs::Event* ev_ptr,
  Transformation &Trans_world_rv,
  EventMatchPair &emPair)
{
  std::vector<std::vector<dvs_msgs::Event*>::iterator> candidates_time_check, candidates_epipolar_check, candidates_motion_check;
  //--- temporal check
  ros::Time time_lowBound(ev_ptr->ts.toSec() - Time_THRESHOLD_ / 2);
  ros::Time time_upBound(ev_ptr->ts.toSec() + Time_THRESHOLD_ / 2);
  std::vector<dvs_msgs::Event*>::iterator candidate_begin_it = EventVecPtr_lower_bound(*pvCandEventPtr_, time_lowBound);
  std::vector<dvs_msgs::Event*>::iterator candidate_end_it = EventVecPtr_lower_bound(*pvCandEventPtr_, time_upBound);

  if (candidate_begin_it != candidate_end_it)
  {
    size_t numCandidate = std::distance(candidate_begin_it, candidate_end_it) + 1;
    candidates_time_check.reserve(numCandidate);
    while (candidate_begin_it != candidate_end_it)
    {
      if ((*candidate_begin_it)->ts.toSec() >= time_lowBound.toSec() &&
          (*candidate_begin_it)->ts.toSec() <= time_upBound.toSec())
      {
        // add polarity check
        if(ev_ptr->polarity == (*candidate_begin_it)->polarity)
          candidates_time_check.emplace_back(candidate_begin_it);
      }
      candidate_begin_it++;
    }
  }
  if (candidates_time_check.size() == 0)
    return false;

  //--- epipolar check
  Eigen::Vector2d x_left = camSysPtr_->cam_left_ptr_->getRectifiedUndistortedCoordinate(ev_ptr->x, ev_ptr->y);
  std::vector<Eigen::Vector2d> xs_right;
  candidates_epipolar_check.reserve(candidates_time_check.size());
  xs_right.reserve(candidates_time_check.size());
  for (size_t i = 0; i < candidates_time_check.size(); i++)
  {
    Eigen::Vector2d x_right = camSysPtr_->cam_right_ptr_->getRectifiedUndistortedCoordinate(
      (*candidates_time_check[i])->x, (*candidates_time_check[i])->y);
    if (fabs(x_left(1) - x_right(1)) <= EPIPOLAR_THRESHOLD_ &&
        x_right(0) < x_left(0))
    {
      candidates_epipolar_check.push_back(candidates_time_check[i]);
      xs_right.push_back(x_right);
    }
  }
  if (candidates_epipolar_check.size() == 0)
    return false;

  //--- motion check (ZNCC)
  double b = camSysPtr_->baseline_;
  double f = camSysPtr_->cam_left_ptr_->P_(0, 0);
  double min_cost = 1.0;// minimum motion inconsistency (eq. 15)
  size_t best_match_id = 0;
  double best_depth = 0;
  Eigen::Matrix<double, 4, 4> T_left_rv =
    pTS_obs_->second.tr_.inverse().getTransformationMatrix() *
    Trans_world_rv.getTransformationMatrix();
  for (size_t i = 0; i < candidates_epipolar_check.size(); i++)
  {
    // triangulation
    double disparity = x_left(0) - xs_right[i](0);
    double depth = b * f / disparity;
    // warping to a pair of TS
    Eigen::Vector2d x1_s;
    Eigen::Vector2d x2_s;
    double cost;
    if (warping2(x_left, 1.0 / depth, T_left_rv.block<3, 4>(0, 0), x1_s, x2_s))
    {
      // extract patch
      Eigen::MatrixXd patch_left, patch_right;
      if (patchInterpolation2(pTS_obs_->second.TS_left_, x1_s, patch_left)
          && patchInterpolation2(pTS_obs_->second.TS_right_, x2_s, patch_right))
      {
        cost = zncc_cost(patch_left, patch_right);
      }
      else
        continue;
    }
    else
      continue;
    // compare
    if (cost < min_cost)
    {
      min_cost = cost;
      best_match_id = i;
      best_depth = depth;
    }
  }
  if (min_cost > TS_NCC_THRESHOLD_)
  {
//      LOG(INFO) << "find no match.";
    return false;
  }

  emPair.x_left_ = x_left;
  emPair.x_right_ = xs_right[best_match_id];
  emPair.t_ = ev_ptr->ts;
  emPair.trans_ = Trans_world_rv;
  emPair.invDepth_ = 1.0 / best_depth;
  emPair.cost_ = min_cost;
  return true;
}

void esvo_core::core::EventMatcher::match_all_SingleThread(
  std::vector<EventMatchPair> &vEMP)
{
  vEMP.reserve(5000);//TODO: find a proper number for the reserve
  for(size_t i = 0;i < pvEventSlice_->size();i++)
  {
    auto it_tmp = (*pvEventSlice_)[i].it_begin_;
    int count = 0;
    while(it_tmp != (*pvEventSlice_)[i].it_end_)
    {
      EventMatchPair emp;
      if(match_an_event(*it_tmp, (*pvEventSlice_)[i].transf_, emp))
        vEMP.push_back(emp);
      it_tmp++;
      count++;
    }
  }
}

void esvo_core::core::EventMatcher::match_all_HyperThread(
  vector<EventMatchPair> &vEMP)
{
  if(pvEventSlice_->size() == 0)
    return;
  // create a vector of job
  std::vector<EventMatcher::Job> jobs(NUM_THREAD_);

  // distribute the workload
  size_t totalNumEvents = 0;
  for(size_t i = 0; i < pvEventSlice_->size(); i++)
    totalNumEvents += (*pvEventSlice_)[i].numEvents_;
  std::vector<size_t> vIndexEventSlice;
  vIndexEventSlice.reserve(totalNumEvents);
  for(size_t i = 0; i < pvEventSlice_->size(); i++)
    vIndexEventSlice.insert(vIndexEventSlice.end(), (*pvEventSlice_)[i].numEvents_, i);

  for(size_t i = 0;i < NUM_THREAD_;i++)
  {
    jobs[i].i_thread_ = i;
    jobs[i].vEventPtr_it_begin_ = (*pvEventSlice_)[0].it_begin_;
    jobs[i].pvEventSlice_ = pvEventSlice_;
    jobs[i].pvIndexEventSlice_ = &vIndexEventSlice;
    jobs[i].pvEventMatchPair_ = std::make_shared<std::vector<EventMatchPair> >();
  }
  // create multiple threads
  std::vector<std::thread> threads;
  threads.reserve(NUM_THREAD_);
  for(size_t i = 0;i < NUM_THREAD_;i++)
    threads.emplace_back(std::bind(&EventMatcher::match, this, jobs[i]));
  for(auto & thread : threads)
  {
    if(thread.joinable())
      thread.join();
  }

  // load result to vEMP
  vEMP.clear();
  for(size_t i = 0; i < NUM_THREAD_; i++)
    vEMP.insert(vEMP.end(), jobs[i].pvEventMatchPair_->begin(), jobs[i].pvEventMatchPair_->end());
}

void esvo_core::core::EventMatcher::match(
  EventMatcher::Job& job)
{
  TicToc tt;
  tt.tic();
  size_t i_thread = job.i_thread_;
  size_t totalNumEvents = job.pvIndexEventSlice_->size();
  job.pvEventMatchPair_->reserve(totalNumEvents / NUM_THREAD_ + 1);

  auto ev_it = job.vEventPtr_it_begin_;
  std::advance(ev_it, i_thread);
  for(size_t i = i_thread; i < totalNumEvents; i+= NUM_THREAD_, std::advance(ev_it, NUM_THREAD_))
  {
    size_t id = (*job.pvIndexEventSlice_)[i];
    EventMatchPair emPair;
    if(match_an_event(*ev_it, (*job.pvEventSlice_)[id].transf_, emPair))
    {
      job.pvEventMatchPair_->push_back(emPair);
    }
  }
}

double esvo_core::core::EventMatcher::zncc_cost(
  Eigen::MatrixXd &patch_left,
  Eigen::MatrixXd &patch_right)
{
  double patch_left_mean = patch_left.mean();
  double patch_right_mean = patch_right.mean();

  Eigen::MatrixXd patch_left_mean_mat = patch_left;
  Eigen::MatrixXd patch_right_mean_mat = patch_right;
  patch_left_mean_mat.setConstant(patch_left_mean);
  patch_right_mean_mat.setConstant(patch_right_mean);

  Eigen::MatrixXd patch_left_sub = patch_left - patch_left_mean_mat;
  Eigen::MatrixXd patch_right_sub = patch_right - patch_right_mean_mat;

  Eigen::MatrixXd patch_left_sub_normalized  = patch_left_sub / (patch_left_sub.norm() + 1e-6);
  Eigen::MatrixXd patch_right_sub_normalized = patch_right_sub / (patch_right_sub.norm() + 1e-6);

  double cost = 0.5 * (1 - (patch_left_sub_normalized.array() * patch_right_sub_normalized.array()).sum());
//  LOG(INFO) << "cost: " << cost;
  return cost;
}

bool esvo_core::core::EventMatcher::warping2(
  const Eigen::Vector2d &x,
  double invDepth,
  const Eigen::Matrix<double, 3, 4> &T_left_rv,
  Eigen::Vector2d &x1_s,
  Eigen::Vector2d &x2_s)
{
  // back-project to 3D
  Eigen::Vector3d p_rv;
  camSysPtr_->cam_left_ptr_->cam2World(x, invDepth, p_rv);
  // transfer to left DVS coordinate
  Eigen::Vector3d p_left = T_left_rv.block<3, 3>(0, 0) * p_rv + T_left_rv.block<3, 1>(0, 3);
  // project onto left and right DVS image plane
  Eigen::Vector3d x1_hom = camSysPtr_->cam_left_ptr_->P_.block<3, 3>(0, 0) * p_left +
                           camSysPtr_->cam_left_ptr_->P_.block<3, 1>(0, 3);
  Eigen::Vector3d x2_hom = camSysPtr_->cam_right_ptr_->P_.block<3, 3>(0, 0) * p_left +
                           camSysPtr_->cam_right_ptr_->P_.block<3, 1>(0, 3);
  x1_s = x1_hom.block<2, 1>(0, 0) / x1_hom(2);
  x2_s = x2_hom.block<2, 1>(0, 0) / x2_hom(2);

  size_t wx = patch_size_X_;
  size_t wy = patch_size_Y_;
  size_t width  = camSysPtr_->cam_left_ptr_->width_;
  size_t height = camSysPtr_->cam_left_ptr_->height_;
  if (x1_s(0) < (wx - 1) / 2 || x1_s(0) > width - (wx - 1) / 2 || x1_s(1) < (wy - 1) / 2 || x1_s(1) > height - (wy - 1) / 2)
    return false;
  if (x2_s(0) < (wx - 1) / 2 || x2_s(0) > width - (wx - 1) / 2 || x2_s(1) < (wy - 1) / 2 || x2_s(1) > height - (wy - 1) / 2)
    return false;
  return true;
}

bool
esvo_core::core::EventMatcher::patchInterpolation2(
  const Eigen::MatrixXd &img,
  const Eigen::Vector2d &location,
  Eigen::MatrixXd &patch)
{
  size_t wx = patch_size_X_;
  size_t wy = patch_size_Y_;
  // compute SrcPatch_UpLeft coordinate and SrcPatch_DownRight coordinate
  // check patch bourndary is inside img boundary
  Eigen::Vector2i SrcPatch_UpLeft, SrcPatch_DownRight;
  SrcPatch_UpLeft << floor(location[0]) - (wx - 1) / 2, floor(location[1]) - (wy - 1) / 2;
  SrcPatch_DownRight << floor(location[0]) + (wx - 1) / 2, floor(location[1]) + (wy - 1) / 2;

  if (SrcPatch_UpLeft[0] < 0 || SrcPatch_UpLeft[1] < 0)
    return false;
  if (SrcPatch_DownRight[0] >= img.cols() || SrcPatch_DownRight[1] >= img.rows())
    return false;

  // compute q1 q2 q3 q4
  Eigen::Vector2d double_indices;
  double_indices << location[1], location[0];

  std::pair<int, int> lower_indices(floor(double_indices[0]), floor(double_indices[1]));
  std::pair<int, int> upper_indices(lower_indices.first + 1, lower_indices.second + 1);

  double q1 = upper_indices.second - double_indices[1];
  double q2 = double_indices[1] - lower_indices.second;
  double q3 = upper_indices.first - double_indices[0];
  double q4 = double_indices[0] - lower_indices.first;

  // extract Src patch, size (wy+1) * (wx+1)
  size_t wx2 = wx + 1;
  size_t wy2 = wy + 1;
  if (SrcPatch_UpLeft[1] + wy >= img.rows() || SrcPatch_UpLeft[0] + wx >= img.cols())
    return false;
  Eigen::MatrixXd SrcPatch = img.block(SrcPatch_UpLeft[1], SrcPatch_UpLeft[0], wy2, wx2);

  // Compute R, size (wy+1) * wx.
  Eigen::MatrixXd R;
  R = q1 * SrcPatch.block(0, 0, wy2, wx) + q2 * SrcPatch.block(0, 1, wy2, wx);

  // Compute F, size wy * wx.
  patch = q3 * R.block(0, 0, wy, wx) + q4 * R.block(1, 0, wy, wx);
  return true;
}