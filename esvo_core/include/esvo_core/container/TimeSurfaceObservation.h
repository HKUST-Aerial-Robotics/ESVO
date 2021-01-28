#ifndef ESVO_CORE_CONTAINER_TIMESURFACEOBSERVATION_H
#define ESVO_CORE_CONTAINER_TIMESURFACEOBSERVATION_H

#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>
#include <cv_bridge/cv_bridge.h>

#include <tf/tf.h>
#include <tf/tfMessage.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <kindr/minimal/quat-transformation.h>

#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <esvo_core/tools/TicToc.h>
#include <esvo_core/tools/utils.h>

//#define TIME_SURFACE_OBSERVATION_LOG
namespace esvo_core
{
using namespace tools;
namespace container
{
struct TimeSurfaceObservation
{
  TimeSurfaceObservation(
    cv_bridge::CvImagePtr &left,
    cv_bridge::CvImagePtr &right,
    Transformation &tr,
    size_t id,
    bool bCalcTsGradient = false)
    : tr_(tr),
      id_(id)
  {
    cv::cv2eigen(left->image, TS_left_);
    cv::cv2eigen(right->image, TS_right_);

    if (bCalcTsGradient)
    {
#ifdef TIME_SURFACE_OBSERVATION_LOG
      TicToc tt;
      tt.tic();
#endif
      cv::Mat cv_dTS_du_left, cv_dTS_dv_left;
      cv::Sobel(left->image, cv_dTS_du_left, CV_64F, 1, 0);
      cv::Sobel(left->image, cv_dTS_dv_left, CV_64F, 0, 1);
      cv::cv2eigen(cv_dTS_du_left, dTS_du_left_);
      cv::cv2eigen(cv_dTS_dv_left, dTS_dv_left_);
#ifdef TIME_SURFACE_OBSERVATION_LOG
      LOG(INFO) << "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@Sobel computation (" << id_ << ") takes " << tt.toc() << " ms.";
#endif
    }
  }

  // override version without initializing the transformation in the constructor.
  TimeSurfaceObservation(
    cv_bridge::CvImagePtr &left,
    cv_bridge::CvImagePtr &right,
    size_t id,
    bool bCalcTsGradient = false)
    : id_(id)
  {
    cvImagePtr_left_ = left;
    cvImagePtr_right_ = right;
    cv::cv2eigen(left->image, TS_left_);
    cv::cv2eigen(right->image, TS_right_);

    if (bCalcTsGradient)
    {
#ifdef TIME_SURFACE_OBSERVATION_LOG
      TicToc tt;
      tt.tic();
#endif
      cv::Mat cv_dTS_du_left, cv_dTS_dv_left;
      cv::Mat cv_dTS_du_right, cv_dTS_dv_right;
      cv::Sobel(left->image, cv_dTS_du_left, CV_64F, 1, 0);
      cv::Sobel(left->image, cv_dTS_dv_left, CV_64F, 0, 1);

      cv::cv2eigen(cv_dTS_du_left, dTS_du_left_);
      cv::cv2eigen(cv_dTS_dv_left, dTS_dv_left_);

#ifdef TIME_SURFACE_OBSERVATION_LOG
      LOG(INFO) << "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@Sobel computation (" << id_ << ") takes " << tt.toc() << " ms.";
#endif
    }
  }

  TimeSurfaceObservation()
  {};

  inline bool isEmpty()
  {
    if(TS_left_.rows() == 0 || TS_left_.cols() == 0 || TS_right_.rows() == 0 || TS_right_.cols() == 0)
      return true;
    else
      return false;
  }

  inline void setTransformation(Transformation &tr)
  {
    tr_ = tr;
  }

  inline void GaussianBlurTS(size_t kernelSize)
  {
    cv::Mat mat_left_, mat_right_;
    cv::GaussianBlur(cvImagePtr_left_->image, mat_left_,
                     cv::Size(kernelSize, kernelSize), 0.0);
    cv::GaussianBlur(cvImagePtr_right_->image, mat_right_,
                     cv::Size(kernelSize, kernelSize), 0.0);
    cv::cv2eigen(mat_left_, TS_left_);
    cv::cv2eigen(mat_right_, TS_right_);
  }

  inline void getTimeSurfaceNegative(size_t kernelSize)
  {
    Eigen::MatrixXd ceilMat(TS_left_.rows(), TS_left_.cols());
    ceilMat.setConstant(255.0);
    if (kernelSize > 0)
    {
      cv::Mat mat_left_;
      cv::GaussianBlur(cvImagePtr_left_->image, mat_left_,
                       cv::Size(kernelSize, kernelSize), 0.0);
      cv::cv2eigen(mat_left_, TS_blurred_left_);
      TS_negative_left_ = ceilMat - TS_blurred_left_;
    }
    else
    {
      TS_negative_left_ = ceilMat - TS_left_;
    }
  }

  inline void computeTsNegativeGrad()
  {
    cv::Mat cv_TS_flipped_left;
    cv::eigen2cv(TS_negative_left_, cv_TS_flipped_left);

    cv::Mat cv_dFlippedTS_du_left, cv_dFlippedTS_dv_left;
    cv::Sobel(cv_TS_flipped_left, cv_dFlippedTS_du_left, CV_64F, 1, 0);
    cv::Sobel(cv_TS_flipped_left, cv_dFlippedTS_dv_left, CV_64F, 0, 1);

    cv::cv2eigen(cv_dFlippedTS_du_left, dTS_negative_du_left_);
    cv::cv2eigen(cv_dFlippedTS_dv_left, dTS_negative_dv_left_);
  }

  Eigen::MatrixXd TS_left_, TS_right_;
  Eigen::MatrixXd TS_blurred_left_;
  Eigen::MatrixXd TS_negative_left_;
  cv_bridge::CvImagePtr cvImagePtr_left_, cvImagePtr_right_;
  Transformation tr_;
  Eigen::MatrixXd dTS_du_left_, dTS_dv_left_;
  Eigen::MatrixXd dTS_negative_du_left_, dTS_negative_dv_left_;
  size_t id_;
};

struct ROSTimeCmp
{
  bool operator()(const ros::Time &a, const ros::Time &b) const
  {
    return a.toNSec() < b.toNSec();
  }
};

using TimeSurfaceHistory = std::map<ros::Time, TimeSurfaceObservation, ROSTimeCmp>;
using StampedTimeSurfaceObs = std::pair<ros::Time, TimeSurfaceObservation>;

inline static TimeSurfaceHistory::iterator TSHistory_lower_bound(TimeSurfaceHistory &ts_history, ros::Time &t)
{
  return std::lower_bound(ts_history.begin(), ts_history.end(), t,
                          [](const std::pair<ros::Time, TimeSurfaceObservation> &tso, const ros::Time &t) {
                            return tso.first.toSec() < t.toSec();
                          });
}

inline static TimeSurfaceHistory::iterator TSHistory_upper_bound(TimeSurfaceHistory &ts_history, ros::Time &t)
{
  return std::upper_bound(ts_history.begin(), ts_history.end(), t,
                          [](const ros::Time &t, const std::pair<ros::Time, TimeSurfaceObservation> &tso) {
                            return t.toSec() < tso.first.toSec();
                          });
}
}
}

#endif //ESVO_CORE_CONTAINER_TIMESURFACEOBSERVATION_H