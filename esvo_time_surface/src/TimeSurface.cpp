#include <esvo_time_surface/TimeSurface.h>
#include <esvo_time_surface/TicToc.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <std_msgs/Float32.h>
#include <glog/logging.h>
#include <thread>

//#define ESVO_TS_LOG

namespace esvo_time_surface 
{
TimeSurface::TimeSurface(ros::NodeHandle & nh, ros::NodeHandle nh_private)
  : nh_(nh)
{
  // setup subscribers and publishers
  event_sub_ = nh_.subscribe("events", 0, &TimeSurface::eventsCallback, this);
  camera_info_sub_ = nh_.subscribe("camera_info", 1, &TimeSurface::cameraInfoCallback, this);
  sync_topic_ = nh_.subscribe("sync", 1, &TimeSurface::syncCallback, this);
  image_transport::ImageTransport it_(nh_);
  time_surface_pub_ = it_.advertise("time_surface", 1);

  // parameters
  nh_private.param<bool>("use_sim_time", bUse_Sim_Time_, true);
  nh_private.param<bool>("ignore_polarity", ignore_polarity_, true);
  nh_private.param<double>("decay_ms", decay_ms_, 30);
  int TS_mode;
  nh_private.param<int>("time_surface_mode", TS_mode, 0);
  time_surface_mode_ = (TimeSurfaceMode)TS_mode;
  nh_private.param<int>("median_blur_kernel_size", median_blur_kernel_size_, 1);
  nh_private.param<int>("max_event_queue_len", max_event_queue_length_, 20);
  //
  bCamInfoAvailable_ = false;
  bSensorInitialized_ = false;
  if(pEventQueueMat_)
    pEventQueueMat_->clear();
  sensor_size_ = cv::Size(0,0);
}

TimeSurface::~TimeSurface()
{
  time_surface_pub_.shutdown();
}

void TimeSurface::init(int width, int height)
{
  sensor_size_ = cv::Size(width, height);
  bSensorInitialized_ = true;
  pEventQueueMat_.reset(new EventQueueMat(width, height, max_event_queue_length_));
  ROS_INFO("Sensor size: (%d x %d)", sensor_size_.width, sensor_size_.height);
}

void TimeSurface::createTimeSurfaceAtTime(const ros::Time& external_sync_time)
{
  std::lock_guard<std::mutex> lock(data_mutex_);

  if(!bSensorInitialized_ || !bCamInfoAvailable_)
    return;

  // create exponential-decayed Time Surface map.
  const double decay_sec = decay_ms_ / 1000.0;
  cv::Mat time_surface_map;
  time_surface_map = cv::Mat::zeros(sensor_size_, CV_64F);

  // Loop through all coordinates
  for(int y=0; y<sensor_size_.height; ++y)
  {
    for(int x=0; x<sensor_size_.width; ++x)
    {
      dvs_msgs::Event most_recent_event_at_coordXY_before_T;
      if(pEventQueueMat_->getMostRecentEventBeforeT(x, y, external_sync_time, &most_recent_event_at_coordXY_before_T))
      {
        const ros::Time& most_recent_stamp_at_coordXY = most_recent_event_at_coordXY_before_T.ts;
        if(most_recent_stamp_at_coordXY.toSec() > 0)
        {
          const double dt = (external_sync_time - most_recent_stamp_at_coordXY).toSec();
          double polarity = (most_recent_event_at_coordXY_before_T.polarity) ? 1.0 : -1.0;
          double expVal = std::exp(-dt / decay_sec);
          if(!ignore_polarity_)
            expVal *= polarity;

          // Backward version
          if(time_surface_mode_ == BACKWARD)
            time_surface_map.at<double>(y,x) = expVal;

          // Forward version
          if(time_surface_mode_ == FORWARD && bCamInfoAvailable_)
          {
            Eigen::Matrix<double, 2, 1> uv_rect = precomputed_rectified_points_.block<2, 1>(0, y * sensor_size_.width + x);
            size_t u_i, v_i;
            if(uv_rect(0) >= 0 && uv_rect(1) >= 0)
            {
              u_i = std::floor(uv_rect(0));
              v_i = std::floor(uv_rect(1));

              if(u_i + 1 < sensor_size_.width && v_i + 1 < sensor_size_.height)
              {
                double fu = uv_rect(0) - u_i;
                double fv = uv_rect(1) - v_i;
                double fu1 = 1.0 - fu;
                double fv1 = 1.0 - fv;
                time_surface_map.at<double>(v_i, u_i) += fu1 * fv1 * expVal;
                time_surface_map.at<double>(v_i, u_i + 1) += fu * fv1 * expVal;
                time_surface_map.at<double>(v_i + 1, u_i) += fu1 * fv * expVal;
                time_surface_map.at<double>(v_i + 1, u_i + 1) += fu * fv * expVal;

                if(time_surface_map.at<double>(v_i, u_i) > 1)
                  time_surface_map.at<double>(v_i, u_i) = 1;
                if(time_surface_map.at<double>(v_i, u_i + 1) > 1)
                  time_surface_map.at<double>(v_i, u_i + 1) = 1;
                if(time_surface_map.at<double>(v_i + 1, u_i) > 1)
                  time_surface_map.at<double>(v_i + 1, u_i) = 1;
                if(time_surface_map.at<double>(v_i + 1, u_i + 1) > 1)
                  time_surface_map.at<double>(v_i + 1, u_i + 1) = 1;
              }
            }
          } // forward
        }
      } // a most recent event is available
    }// loop x
  }// loop y

  // polarity
  if(!ignore_polarity_)
    time_surface_map = 255.0 * (time_surface_map + 1.0) / 2.0;
  else
    time_surface_map = 255.0 * time_surface_map;
  time_surface_map.convertTo(time_surface_map, CV_8U);

  // median blur
  if(median_blur_kernel_size_ > 0)
    cv::medianBlur(time_surface_map, time_surface_map, 2 * median_blur_kernel_size_ + 1);

  // Publish event image
  static cv_bridge::CvImage cv_image;
  cv_image.encoding = "mono8";
  cv_image.image = time_surface_map.clone();

  if(time_surface_mode_ == FORWARD && time_surface_pub_.getNumSubscribers() > 0)
  {
    cv_image.header.stamp = external_sync_time;
    time_surface_pub_.publish(cv_image.toImageMsg());
  }

  if (time_surface_mode_ == BACKWARD && bCamInfoAvailable_ && time_surface_pub_.getNumSubscribers() > 0)
  {
    cv_bridge::CvImage cv_image2;
    cv_image2.encoding = cv_image.encoding;
    cv_image2.header.stamp = external_sync_time;
    cv::remap(cv_image.image, cv_image2.image, undistort_map1_, undistort_map2_, CV_INTER_LINEAR);
    time_surface_pub_.publish(cv_image2.toImageMsg());
  }
}

void TimeSurface::createTimeSurfaceAtTime_hyperthread(const ros::Time& external_sync_time)
{
  std::lock_guard<std::mutex> lock(data_mutex_);

  if(!bSensorInitialized_ || !bCamInfoAvailable_)
    return;

  // create exponential-decayed Time Surface map.
  const double decay_sec = decay_ms_ / 1000.0;
  cv::Mat time_surface_map;
  time_surface_map = cv::Mat::zeros(sensor_size_, CV_64F);

  // distribute jobs
  std::vector<Job> jobs(NUM_THREAD_TS);
  size_t num_col_per_thread = sensor_size_.width / NUM_THREAD_TS;
  size_t res_col = sensor_size_.width % NUM_THREAD_TS;
  for(size_t i = 0; i < NUM_THREAD_TS; i++)
  {
    jobs[i].i_thread_ = i;
    jobs[i].pEventQueueMat_ = pEventQueueMat_.get();
    jobs[i].pTimeSurface_ = &time_surface_map;
    jobs[i].start_col_ = num_col_per_thread * i;
    if(i == NUM_THREAD_TS - 1)
      jobs[i].end_col_ = jobs[i].start_col_ + num_col_per_thread - 1 + res_col;
    else
      jobs[i].end_col_ = jobs[i].start_col_ + num_col_per_thread - 1;
    jobs[i].start_row_ = 0;
    jobs[i].end_row_ = sensor_size_.height - 1;
    jobs[i].external_sync_time_ = external_sync_time;
    jobs[i].decay_sec_ = decay_sec;
  }

  // hyper thread processing
  std::vector<std::thread> threads;
  threads.reserve(NUM_THREAD_TS);
  for(size_t i = 0; i < NUM_THREAD_TS; i++)
    threads.emplace_back(std::bind(&TimeSurface::thread, this, jobs[i]));
  for(auto& thread:threads)
    if(thread.joinable())
      thread.join();

  // polarity
  if(!ignore_polarity_)
    time_surface_map = 255.0 * (time_surface_map + 1.0) / 2.0;
  else
    time_surface_map = 255.0 * time_surface_map;
  time_surface_map.convertTo(time_surface_map, CV_8U);

  // median blur
  if(median_blur_kernel_size_ > 0)
    cv::medianBlur(time_surface_map, time_surface_map, 2 * median_blur_kernel_size_ + 1);

  // Publish event image
  static cv_bridge::CvImage cv_image;
  cv_image.encoding = "mono8";
  cv_image.image = time_surface_map.clone();

  if(time_surface_mode_ == FORWARD && time_surface_pub_.getNumSubscribers() > 0)
  {
    cv_image.header.stamp = external_sync_time;
    time_surface_pub_.publish(cv_image.toImageMsg());
  }

  if (time_surface_mode_ == BACKWARD && bCamInfoAvailable_ && time_surface_pub_.getNumSubscribers() > 0)
  {
    cv_bridge::CvImage cv_image2;
    cv_image2.encoding = cv_image.encoding;
    cv_image2.header.stamp = external_sync_time;
    cv::remap(cv_image.image, cv_image2.image, undistort_map1_, undistort_map2_, CV_INTER_LINEAR);
    time_surface_pub_.publish(cv_image2.toImageMsg());
  }
}

void TimeSurface::thread(Job &job)
{
  EventQueueMat & eqMat = *job.pEventQueueMat_;
  cv::Mat& time_surface_map = *job.pTimeSurface_;
  size_t start_col = job.start_col_;
  size_t end_col = job.end_col_;
  size_t start_row = job.start_row_;
  size_t end_row = job.end_row_;
  size_t i_thread = job.i_thread_;

  for(size_t y = start_row; y <= end_row; y++)
    for(size_t x = start_col; x <= end_col; x++)
    {
      dvs_msgs::Event most_recent_event_at_coordXY_before_T;
      if(pEventQueueMat_->getMostRecentEventBeforeT(x, y, job.external_sync_time_, &most_recent_event_at_coordXY_before_T))
      {
        const ros::Time& most_recent_stamp_at_coordXY = most_recent_event_at_coordXY_before_T.ts;
        if(most_recent_stamp_at_coordXY.toSec() > 0)
        {
          const double dt = (job.external_sync_time_ - most_recent_stamp_at_coordXY).toSec();
          double polarity = (most_recent_event_at_coordXY_before_T.polarity) ? 1.0 : -1.0;
          double expVal = std::exp(-dt / job.decay_sec_);
          if(!ignore_polarity_)
            expVal *= polarity;

          // Backward version
          if(time_surface_mode_ == BACKWARD)
            time_surface_map.at<double>(y,x) = expVal;

          // Forward version
          if(time_surface_mode_ == FORWARD && bCamInfoAvailable_)
          {
            Eigen::Matrix<double, 2, 1> uv_rect = precomputed_rectified_points_.block<2, 1>(0, y * sensor_size_.width + x);
            size_t u_i, v_i;
            if(uv_rect(0) >= 0 && uv_rect(1) >= 0)
            {
              u_i = std::floor(uv_rect(0));
              v_i = std::floor(uv_rect(1));

              if(u_i + 1 < sensor_size_.width && v_i + 1 < sensor_size_.height)
              {
                double fu = uv_rect(0) - u_i;
                double fv = uv_rect(1) - v_i;
                double fu1 = 1.0 - fu;
                double fv1 = 1.0 - fv;
                time_surface_map.at<double>(v_i, u_i) += fu1 * fv1 * expVal;
                time_surface_map.at<double>(v_i, u_i + 1) += fu * fv1 * expVal;
                time_surface_map.at<double>(v_i + 1, u_i) += fu1 * fv * expVal;
                time_surface_map.at<double>(v_i + 1, u_i + 1) += fu * fv * expVal;

                if(time_surface_map.at<double>(v_i, u_i) > 1)
                  time_surface_map.at<double>(v_i, u_i) = 1;
                if(time_surface_map.at<double>(v_i, u_i + 1) > 1)
                  time_surface_map.at<double>(v_i, u_i + 1) = 1;
                if(time_surface_map.at<double>(v_i + 1, u_i) > 1)
                  time_surface_map.at<double>(v_i + 1, u_i) = 1;
                if(time_surface_map.at<double>(v_i + 1, u_i + 1) > 1)
                  time_surface_map.at<double>(v_i + 1, u_i + 1) = 1;
              }
            }
          } // forward
        }
      } // a most recent event is available
    }
}

void TimeSurface::syncCallback(const std_msgs::TimeConstPtr& msg)
{
  if(bUse_Sim_Time_)
    sync_time_ = ros::Time::now();
  else
    sync_time_ = msg->data;

#ifdef ESVO_TS_LOG
    TicToc tt;
    tt.tic();
#endif
    if(NUM_THREAD_TS == 1)
      createTimeSurfaceAtTime(sync_time_);
    if(NUM_THREAD_TS > 1)
      createTimeSurfaceAtTime_hyperthread(sync_time_);
#ifdef ESVO_TS_LOG
    LOG(INFO) << "Time Surface map's creation takes: " << tt.toc() << " ms.";
#endif
}

void TimeSurface::cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg)
{
  if(bCamInfoAvailable_)
    return;

  cv::Size sensor_size(msg->width, msg->height);
  camera_matrix_ = cv::Mat(3, 3, CV_64F);
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      camera_matrix_.at<double>(cv::Point(i, j)) = msg->K[i+j*3];

  distortion_model_ = msg->distortion_model;
  dist_coeffs_ = cv::Mat(msg->D.size(), 1, CV_64F);
  for (int i = 0; i < msg->D.size(); i++)
    dist_coeffs_.at<double>(i) = msg->D[i];

  rectification_matrix_ = cv::Mat(3, 3, CV_64F);
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      rectification_matrix_.at<double>(cv::Point(i, j)) = msg->R[i+j*3];

  projection_matrix_ = cv::Mat(3, 4, CV_64F);
  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 3; j++)
      projection_matrix_.at<double>(cv::Point(i, j)) = msg->P[i+j*4];

  if(distortion_model_ == "equidistant")
  {
    cv::fisheye::initUndistortRectifyMap(camera_matrix_, dist_coeffs_,
                                         rectification_matrix_, projection_matrix_,
                                         sensor_size, CV_32FC1, undistort_map1_, undistort_map2_);
    bCamInfoAvailable_ = true;
    ROS_INFO("Camera information is loaded (Distortion model %s).", distortion_model_.c_str());
  }
  else if(distortion_model_ == "plumb_bob")
  {
    cv::initUndistortRectifyMap(camera_matrix_, dist_coeffs_,
                                rectification_matrix_, projection_matrix_,
                                sensor_size, CV_32FC1, undistort_map1_, undistort_map2_);
    bCamInfoAvailable_ = true;
    ROS_INFO("Camera information is loaded (Distortion model %s).", distortion_model_.c_str());
  }
  else
  {
    ROS_ERROR_ONCE("Distortion model %s is not supported.", distortion_model_.c_str());
    bCamInfoAvailable_ = false;
    return;
  }

  /* pre-compute the undistorted-rectified look-up table */
  precomputed_rectified_points_ = Eigen::Matrix2Xd(2, sensor_size.height * sensor_size.width);
  // raw coordinates
  cv::Mat_<cv::Point2f> RawCoordinates(1, sensor_size.height * sensor_size.width);
  for (int y = 0; y < sensor_size.height; y++)
  {
    for (int x = 0; x < sensor_size.width; x++)
    {
      int index = y * sensor_size.width + x;
      RawCoordinates(index) = cv::Point2f((float) x, (float) y);
    }
  }
  // undistorted-rectified coordinates
  cv::Mat_<cv::Point2f> RectCoordinates(1, sensor_size.height * sensor_size.width);
  if (distortion_model_ == "plumb_bob")
  {
    cv::undistortPoints(RawCoordinates, RectCoordinates, camera_matrix_, dist_coeffs_,
                        rectification_matrix_, projection_matrix_);
    ROS_INFO("Undistorted-Rectified Look-Up Table with Distortion model: %s", distortion_model_.c_str());
  }
  else if (distortion_model_ == "equidistant")
  {
    cv::fisheye::undistortPoints(
      RawCoordinates, RectCoordinates, camera_matrix_, dist_coeffs_,
      rectification_matrix_, projection_matrix_);
    ROS_INFO("Undistorted-Rectified Look-Up Table with Distortion model: %s", distortion_model_.c_str());
  }
  else
  {
    std::cout << "Unknown distortion model is provided." << std::endl;
    exit(-1);
  }
  // load look-up table
  for (size_t i = 0; i < sensor_size.height * sensor_size.width; i++)
  {
    precomputed_rectified_points_.col(i) = Eigen::Matrix<double, 2, 1>(
      RectCoordinates(i).x, RectCoordinates(i).y);
  }
  ROS_INFO("Undistorted-Rectified Look-Up Table has been computed.");
}

void TimeSurface::eventsCallback(const dvs_msgs::EventArray::ConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);

  if(!bSensorInitialized_)
    init(msg->width, msg->height);

  for(const dvs_msgs::Event& e : msg->events)
    pEventQueueMat_->insertEvent(e);
}

} // namespace esvo_time_surface