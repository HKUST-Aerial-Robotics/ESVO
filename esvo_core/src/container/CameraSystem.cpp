#include <esvo_core/container/CameraSystem.h>
#include <opencv2/core/eigen.hpp>
#include <glog/logging.h>

namespace esvo_core
{
namespace container
{
PerspectiveCamera::PerspectiveCamera()
{
}
PerspectiveCamera::~PerspectiveCamera() {}

void PerspectiveCamera::setIntrinsicParameters(
  size_t width,
  size_t height,
  std::string &cameraName,
  std::string &distortion_model,
  std::vector<double> &vD,
  std::vector<double> &vK,
  std::vector<double> &vRectMat,
  std::vector<double> &vP)
{
  width_ = width;
  height_ = height;
  cameraName_ = cameraName;
  distortion_model_ = distortion_model;
  D_ = Eigen::Matrix<double,4,1>(vD.data());
  K_ = Eigen::Matrix<double,3,3,Eigen::RowMajor>(vK.data());
  RectMat_ = Eigen::Matrix<double,3,3,Eigen::RowMajor>(vRectMat.data());
  P_ = Eigen::Matrix<double,3,4,Eigen::RowMajor>(vP.data());

  preComputeRectifiedCoordinate();
}

void PerspectiveCamera::preComputeRectifiedCoordinate()
{
  precomputed_rectified_points_ = Eigen::Matrix2Xd(2, height_ * width_);

  cv::Mat_<cv::Point2f> RawCoordinates(1, width_ * height_);
  for (int y = 0; y < height_; y++)
  {
    for (int x = 0; x < width_; x++)
    {
      int index = y * width_ + x;
      RawCoordinates(index) = cv::Point2f((float) x, (float) y);
    }
  }

  cv::Mat_<cv::Point2f> RectCoordinates(1, height_ * width_);
  cv::Mat cvKmat(3, 3, CV_64F);
  cv::Mat cvDistCoeff(1, 4, CV_64F);
  cv::Mat cvRectMat(3, 3, CV_64F);
  cv::Mat cvPmat(3, 4, CV_64F);

  cv::eigen2cv(K_, cvKmat);
  cv::eigen2cv(D_, cvDistCoeff);
  cv::eigen2cv(RectMat_, cvRectMat);
  cv::eigen2cv(P_, cvPmat);
  if (distortion_model_ == "plumb_bob")
  {
    cv::undistortPoints(RawCoordinates, RectCoordinates, cvKmat, cvDistCoeff, cvRectMat, cvPmat);
#if CV_MAJOR_VERSION >= 3
    cv::Size sensor_size(width_, height_);
    cv::initUndistortRectifyMap(cvKmat, cvDistCoeff,cvRectMat, cvPmat,
       sensor_size, CV_32FC1, undistort_map1_, undistort_map2_);
    cv::Mat cvSrcMask = cvSrcMask.ones(height_, width_, CV_32F);
    cv::Mat cvDstMask = cvSrcMask.zeros(height_, width_, CV_32F);
    cv::remap(cvSrcMask, cvDstMask, undistort_map1_, undistort_map2_, CV_INTER_LINEAR);
    cv::threshold(cvDstMask, cvDstMask, 0.999, 255, cv::THRESH_BINARY);
    cvDstMask.convertTo(cvDstMask, CV_8U);
    cv::cv2eigen(cvDstMask, UndistortRectify_mask_);
//    LOG(INFO) << "#################### UndistortRectify_mask_.size: " << UndistortRectify_mask_.size();
#else
    ROS_ERROR_ONCE("You need OpenCV >= 3.0 to use the equidistant camera model.");
    ROS_ERROR_ONCE("Will not publish rectified images.");
#endif
  }
  else if (distortion_model_ == "equidistant")
  {
    cv::fisheye::undistortPoints(
      RawCoordinates, RectCoordinates, cvKmat, cvDistCoeff, cvRectMat, cvPmat);
#if CV_MAJOR_VERSION >= 3
    cv::Size sensor_size(width_, height_);
    cv::fisheye::initUndistortRectifyMap(cvKmat, cvDistCoeff, cvRectMat, cvPmat,
                   sensor_size, CV_32FC1, undistort_map1_, undistort_map2_);
    cv::Mat cvSrcMask = cvSrcMask.ones(height_, width_, CV_32F);
    cv::Mat cvDstMask = cvSrcMask.zeros(height_, width_, CV_32F);
    cv::remap(cvSrcMask, cvDstMask, undistort_map1_, undistort_map2_, CV_INTER_LINEAR);
    cv::threshold(cvDstMask, cvDstMask, 0.1, 255, cv::THRESH_BINARY);
    cvDstMask.convertTo(cvDstMask, CV_8U);
    cv::cv2eigen(cvDstMask, UndistortRectify_mask_);

//    LOG(INFO) << "#################### UndistortRectify_mask_.size: " << UndistortRectify_mask_.size();
#else
    ROS_ERROR_ONCE("You need OpenCV >= 3.0 to use the equidistant camera model.");
    ROS_ERROR_ONCE("Will not publish rectified images.");
#endif
  }
  else
  {
    std::cout << "wrong distortion model is provided." << std::endl;
    exit(-1);
  }

  for (size_t i = 0; i < height_ * width_; i++)
  {
    precomputed_rectified_points_.col(i) = Eigen::Matrix<double, 2, 1>(
      RectCoordinates(i).x, RectCoordinates(i).y);
  }
}

Eigen::Matrix<double, 2, 1>
PerspectiveCamera::getRectifiedUndistortedCoordinate(int xcoor, int ycoor)
{
  size_t index = ycoor * width_ + xcoor;
  return precomputed_rectified_points_.block<2, 1>(0, index);
}

void
PerspectiveCamera::cam2World(
  const Eigen::Vector2d &x,
  double invDepth,
  Eigen::Vector3d &p)
{
  double z = 1.0 / invDepth;
  Eigen::Matrix<double, 4, 1> x_ss;
  x_ss << x(0),
    x(1),
    1,
    1;
  Eigen::Vector4d temp;
  temp << 0, 0, 0, z;// this z is correct
  Eigen::Matrix<double, 4, 4> P_tilde;
  P_tilde.block<3, 4>(0, 0) = P_;
  P_tilde.block<1, 4>(3, 0) = temp;
  Eigen::Vector4d p_s = z * P_tilde.inverse() * x_ss;
  p = p_s.block<3, 1>(0, 0) / p_s(3); // This normalization by the last element is not necessary.
}

void
PerspectiveCamera::world2Cam(
  const Eigen::Vector3d &p,
  Eigen::Vector2d &x)
{
  Eigen::Vector3d x_hom = P_.block<3, 3>(0, 0) * p + P_.block<3, 1>(0, 3);
  x = x_hom.head(2) / x_hom(2);
}

/************************************************************/
/************************************************************/
CameraSystem::CameraSystem(const std::string& calibInfoDir, bool bPrintCalibInfo)
{
  cam_left_ptr_ = std::shared_ptr<PerspectiveCamera>(new PerspectiveCamera());
  cam_right_ptr_ = std::shared_ptr<PerspectiveCamera>(new PerspectiveCamera());
  loadCalibInfo(calibInfoDir, bPrintCalibInfo);
  computeBaseline();
}
CameraSystem::~CameraSystem() {}

void CameraSystem::computeBaseline()
{
  Eigen::Vector3d temp = cam_right_ptr_->P_.block<3,3>(0,0).inverse() *
    cam_right_ptr_->P_.block<3,1>(0,3);
  baseline_ = temp.norm();
}

void CameraSystem::loadCalibInfo(const std::string &cameraSystemDir, bool bPrintCalibInfo)
{
  const std::string left_cam_calib_dir(cameraSystemDir + "/left.yaml");
  const std::string right_cam_calib_dir(cameraSystemDir + "/right.yaml");
  YAML::Node leftCamCalibInfo = YAML::LoadFile(left_cam_calib_dir);
  YAML::Node rightCamCalibInfo = YAML::LoadFile(right_cam_calib_dir);

  // load calib (left)
  size_t width = leftCamCalibInfo["image_width"].as<int>();
  size_t height = leftCamCalibInfo["image_height"].as<int>();
  std::string cameraNameLeft = leftCamCalibInfo["camera_name"].as<std::string>();
  std::string cameraNameRight = rightCamCalibInfo["camera_name"].as<std::string>();
  std::string distortion_model = leftCamCalibInfo["distortion_model"].as<std::string>();
  std::vector<double> vD_left, vK_left, vRectMat_left, vP_left;
  std::vector<double> vD_right, vK_right, vRectMat_right, vP_right;
  std::vector<double> vT_right_left;

  vD_left = leftCamCalibInfo["distortion_coefficients"]["data"].as< std::vector<double> >();
  vK_left = leftCamCalibInfo["camera_matrix"]["data"].as< std::vector<double> >();
  vRectMat_left = leftCamCalibInfo["rectification_matrix"]["data"].as< std::vector<double> >();
  vP_left = leftCamCalibInfo["projection_matrix"]["data"].as< std::vector<double> >();

  vD_right = rightCamCalibInfo["distortion_coefficients"]["data"].as< std::vector<double> >();
  vK_right = rightCamCalibInfo["camera_matrix"]["data"].as< std::vector<double> >();
  vRectMat_right = rightCamCalibInfo["rectification_matrix"]["data"].as< std::vector<double> >();
  vP_right = rightCamCalibInfo["projection_matrix"]["data"].as< std::vector<double> >();

  vT_right_left = leftCamCalibInfo["T_right_left"]["data"].as< std::vector<double> >();

  cam_left_ptr_->setIntrinsicParameters(
    width, height,
    cameraNameLeft,
    distortion_model,
    vD_left, vK_left, vRectMat_left, vP_left);
  cam_right_ptr_->setIntrinsicParameters(
    width, height,
    cameraNameRight,
    distortion_model,
    vD_right, vK_right, vRectMat_right, vP_right);

  T_right_left_ = Eigen::Matrix<double,3,4,Eigen::RowMajor>(vT_right_left.data());

  if(bPrintCalibInfo)
    printCalibInfo();
}

void CameraSystem::printCalibInfo()
{
  LOG(INFO) << "============================================" << std::endl;
  LOG(INFO) << "Left Camera" << std::endl;
  LOG(INFO) << "--image_width: " << cam_left_ptr_->width_;
  LOG(INFO) << "--image_height: " << cam_left_ptr_->height_;
  LOG(INFO) << "--distortion model: " << cam_left_ptr_->distortion_model_;
  LOG(INFO) << "--distortion_coefficients:\n" << cam_left_ptr_->D_;
  LOG(INFO) << "--rectification_matrix:\n" <<  cam_left_ptr_->RectMat_;
  LOG(INFO) << "--projection_matrix:\n" <<  cam_left_ptr_->P_;
  LOG(INFO) << "--T_right_left:\n" <<  T_right_left_;

  LOG(INFO) << "============================================" << std::endl;
  LOG(INFO) << "Right Camera:" << std::endl;
  LOG(INFO) << "--image_width: " << cam_right_ptr_->width_;
  LOG(INFO) << "--image_height: " << cam_right_ptr_->height_;
  LOG(INFO) << "--distortion model:" << cam_right_ptr_->distortion_model_;
  LOG(INFO) << "--distortion_coefficients:\n" << cam_right_ptr_->D_;
  LOG(INFO) << "--rectification_matrix:\n" <<  cam_right_ptr_->RectMat_;
  LOG(INFO) << "--projection_matrix:\n" <<  cam_right_ptr_->P_;
  LOG(INFO) << "============================================" << std::endl;
}

} // container

} //esvo_core