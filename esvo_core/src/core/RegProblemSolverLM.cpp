#include <esvo_core/core/RegProblemSolverLM.h>
#include <esvo_core/tools/cayley.h>

namespace esvo_core
{
namespace core
{
RegProblemSolverLM::RegProblemSolverLM(
  esvo_core::CameraSystem::Ptr &camSysPtr,
  shared_ptr<RegProblemConfig> &rpConfigPtr,
  esvo_core::core::RegProblemType rpType,
  size_t numThread):
  camSysPtr_(camSysPtr),
  rpConfigPtr_(rpConfigPtr),
  rpType_(rpType),
  NUM_THREAD_(numThread),
  bPrint_(false),
  bVisualize_(true)
{
  if(rpType_ == REG_NUMERICAL)
  {
    numDiff_regProblemPtr_ =
      std::make_shared<Eigen::NumericalDiff<RegProblemLM> >(camSysPtr_, rpConfigPtr_, NUM_THREAD_);
  }
  else if(rpType_ == REG_ANALYTICAL)
  {
    regProblemPtr_ = std::make_shared<RegProblemLM>(camSysPtr_, rpConfigPtr_, NUM_THREAD_);
  }
  else
  {
    LOG(ERROR) << "Wrong Registration Problem Type is assigned!!!";
    exit(-1);
  }
  z_min_ = 1.0 / rpConfigPtr_->invDepth_max_range_;
  z_max_ = 1.0 / rpConfigPtr_->invDepth_min_range_;

  lmStatics_.nPoints_ = 0;
  lmStatics_.nfev_ = 0;
  lmStatics_.nIter_ = 0;
}

RegProblemSolverLM::~RegProblemSolverLM()
{}

bool RegProblemSolverLM::resetRegProblem(RefFrame* ref, CurFrame* cur)
{
  if(cur->numEventsSinceLastObs_ < rpConfigPtr_->MIN_NUM_EVENTS_)
  {
    LOG(INFO) << "resetRegProblem RESET fails for no enough events coming in.";
    LOG(INFO) << "However, the system remains to work.";
  }
  if( ref->vPointXYZPtr_.size() < rpConfigPtr_->BATCH_SIZE_ )
  {
    LOG(INFO) << "resetRegProblem RESET fails for no enough point cloud in the local map.";
    LOG(INFO) << "The system will be re-initialized";
    return false;
  }
  //  LOG(INFO) << "resetRegProblem RESET succeeds.";
  if(rpType_ == REG_NUMERICAL)
  {
    numDiff_regProblemPtr_->setProblem(ref, cur, false);
//    LOG(INFO) << "numDiff_regProblemPtr_->setProblem(ref, cur, false) -----------------";
  }
  if(rpType_ == REG_ANALYTICAL)
  {
    regProblemPtr_->setProblem(ref, cur, true);
//    LOG(INFO) << "regProblemPtr_->setProblem(ref, cur, true) -----------------";
  }

  lmStatics_.nPoints_ = 0;
  lmStatics_.nfev_ = 0;
  lmStatics_.nIter_ = 0;
  return true;
}

bool RegProblemSolverLM::solve_numerical()
{
  Eigen::LevenbergMarquardt<Eigen::NumericalDiff<RegProblemLM>, double> lm(*numDiff_regProblemPtr_.get());
  lm.resetParameters();
  lm.parameters.ftol = 1e-3;
  lm.parameters.xtol = 1e-3;
  lm.parameters.maxfev = rpConfigPtr_->MAX_ITERATION_ * 8;

  size_t iteration = 0;
  size_t nfev = 0;
  while(true)
  {
    if(iteration >= rpConfigPtr_->MAX_ITERATION_)
      break;
    numDiff_regProblemPtr_->setStochasticSampling(
      (iteration % numDiff_regProblemPtr_->numBatches_) * rpConfigPtr_->BATCH_SIZE_, rpConfigPtr_->BATCH_SIZE_);
    Eigen::VectorXd x(6);
    x.fill(0.0);
    if(lm.minimizeInit(x) == Eigen::LevenbergMarquardtSpace::ImproperInputParameters)
    {
      LOG(ERROR) << "ImproperInputParameters for LM (Tracking)." << std::endl;
      return false;
    }

    Eigen::LevenbergMarquardtSpace::Status status = lm.minimizeOneStep(x);
    numDiff_regProblemPtr_->addMotionUpdate(x);

    iteration++;
    nfev += lm.nfev;

    /*************************** Visualization ************************/
    if(bVisualize_)// will slow down the tracker's performance a little bit
    {
      size_t width = camSysPtr_->cam_left_ptr_->width_;
      size_t height = camSysPtr_->cam_left_ptr_->height_;
      cv::Mat reprojMap_left = cv::Mat(cv::Size(width, height), CV_8UC1, cv::Scalar(0));
      cv::eigen2cv(numDiff_regProblemPtr_->cur_->pTsObs_->TS_negative_left_, reprojMap_left);
      reprojMap_left.convertTo(reprojMap_left, CV_8UC1);
      cv::cvtColor(reprojMap_left, reprojMap_left, CV_GRAY2BGR);

      // project 3D points to current frame
      Eigen::Matrix3d R_cur_ref =  numDiff_regProblemPtr_->R_.transpose();
      Eigen::Vector3d t_cur_ref = -numDiff_regProblemPtr_->R_.transpose() * numDiff_regProblemPtr_->t_;

      size_t numVisualization = std::min(numDiff_regProblemPtr_->ResItems_.size(), (size_t)2000);
      for(size_t i = 0; i < numVisualization; i++)
      {
        ResidualItem & ri = numDiff_regProblemPtr_->ResItems_[i];
        Eigen::Vector3d p_3D = R_cur_ref * ri.p_ + t_cur_ref;
        Eigen::Vector2d p_img_left;
        camSysPtr_->cam_left_ptr_->world2Cam(p_3D, p_img_left);
        double z = ri.p_[2];
        visualizor_.DrawPoint(1.0 / z, 1.0 / z_min_, 1.0 / z_max_,
                              Eigen::Vector2d(p_img_left(0), p_img_left(1)), reprojMap_left);
      }
      std_msgs::Header header;
      header.stamp = numDiff_regProblemPtr_->cur_->t_;
      sensor_msgs::ImagePtr msg = cv_bridge::CvImage(header, "bgr8", reprojMap_left).toImageMsg();
      reprojMap_pub_->publish(msg);
    }
    /*************************** Visualization ************************/
    if(status == 2 || status == 3)
      break;
  }
//  LOG(INFO) << "LM Finished ...................";
  numDiff_regProblemPtr_->setPose();
  lmStatics_.nPoints_ = numDiff_regProblemPtr_->numPoints_;
  lmStatics_.nfev_ = nfev;
  lmStatics_.nIter_ = iteration;
}

bool RegProblemSolverLM::solve_analytical()
{
  Eigen::LevenbergMarquardt<RegProblemLM, double> lm(*regProblemPtr_.get());
  lm.resetParameters();
  lm.parameters.ftol = 1e-3;
  lm.parameters.xtol = 1e-3;
  lm.parameters.maxfev = rpConfigPtr_->MAX_ITERATION_ * 8;

  size_t iteration = 0;
  size_t nfev = 0;
  while(true)
  {
    if(iteration >= rpConfigPtr_->MAX_ITERATION_)
      break;
    regProblemPtr_->setStochasticSampling(
      (iteration % regProblemPtr_->numBatches_) * rpConfigPtr_->BATCH_SIZE_, rpConfigPtr_->BATCH_SIZE_);
    Eigen::VectorXd x(6);
    x.fill(0.0);
    if(lm.minimizeInit(x) == Eigen::LevenbergMarquardtSpace::ImproperInputParameters)
    {
      LOG(ERROR) << "ImproperInputParameters for LM (Tracking)." << std::endl;
      return false;
    }
    Eigen::LevenbergMarquardtSpace::Status status = lm.minimizeOneStep(x);
    regProblemPtr_->addMotionUpdate(x);

    iteration++;
    nfev += lm.nfev;
    if(status == 2 || status == 3)
      break;
  }

  /*************************** Visualization ************************/
  if(bVisualize_) // will slow down the tracker a little bit
  {
    size_t width = camSysPtr_->cam_left_ptr_->width_;
    size_t height = camSysPtr_->cam_left_ptr_->height_;
    cv::Mat reprojMap_left = cv::Mat(cv::Size(width, height), CV_8UC1, cv::Scalar(0));
    cv::eigen2cv(regProblemPtr_->cur_->pTsObs_->TS_negative_left_, reprojMap_left);
    reprojMap_left.convertTo(reprojMap_left, CV_8UC1);
    cv::cvtColor(reprojMap_left, reprojMap_left, CV_GRAY2BGR);

    // project 3D points to current frame
    Eigen::Matrix3d R_cur_ref =  regProblemPtr_->R_.transpose();
    Eigen::Vector3d t_cur_ref = -regProblemPtr_->R_.transpose() * regProblemPtr_->t_;

    size_t numVisualization = std::min(regProblemPtr_->ResItems_.size(), (size_t)2000);
    for(size_t i = 0; i < numVisualization; i++)
    {
      ResidualItem & ri = regProblemPtr_->ResItems_[i];
      Eigen::Vector3d p_3D = R_cur_ref * ri.p_ + t_cur_ref;
      Eigen::Vector2d p_img_left;
      camSysPtr_->cam_left_ptr_->world2Cam(p_3D, p_img_left);
      double z = ri.p_[2];
      visualizor_.DrawPoint(1.0 / z, 1.0 / z_min_, 1.0 / z_max_,
                            Eigen::Vector2d(p_img_left(0), p_img_left(1)), reprojMap_left);
    }
    std_msgs::Header header;
    header.stamp = regProblemPtr_->cur_->t_;
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(header, "bgr8", reprojMap_left).toImageMsg();
    reprojMap_pub_->publish(msg);
  }
  /*************************** Visualization ************************/

  regProblemPtr_->setPose();
  lmStatics_.nPoints_ = regProblemPtr_->numPoints_;
  lmStatics_.nfev_ = nfev;
  lmStatics_.nIter_ = iteration;
}

void RegProblemSolverLM::setRegPublisher(
  image_transport::Publisher* reprojMap_pub)
{
  reprojMap_pub_ = reprojMap_pub;
}

}//namespace core
}//namespace esvo_core

