#ifndef ESVO_CORE_CONTAINER_CAMERASYSTEM_H
#define ESVO_CORE_CONTAINER_CAMERASYSTEM_H

#include <string>
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
#include <boost/shared_ptr.hpp>
#include <yaml-cpp/yaml.h>

namespace esvo_core
{
namespace container
{
class PerspectiveCamera
{
  public:
  PerspectiveCamera();
  virtual ~PerspectiveCamera();
  using Ptr = std::shared_ptr<PerspectiveCamera>;

  void setIntrinsicParameters(
    size_t width, size_t height,
    std::string& cameraName,
    std::string& distortion_model,
    std::vector<double>& vD,
    std::vector<double>& vK,
    std::vector<double>& vRectMat,
    std::vector<double>& vP);

  void preComputeRectifiedCoordinate();

  Eigen::Matrix<double, 2, 1> getRectifiedUndistortedCoordinate(int xcoor, int ycoor);

  void cam2World(const Eigen::Vector2d &x, double invDepth, Eigen::Vector3d &p);

  void world2Cam(const Eigen::Vector3d &p, Eigen::Vector2d &x);

  public:
  size_t width_, height_;
  std::string cameraName_;
  std::string distortion_model_;
  Eigen::Matrix<double, 4, 1> D_;
  Eigen::Matrix3d K_;
  Eigen::Matrix3d RectMat_;
  Eigen::Matrix<double, 3, 4> P_;

  Eigen::Matrix2Xd precomputed_rectified_points_;
  cv::Mat undistort_map1_, undistort_map2_;
  Eigen::MatrixXi UndistortRectify_mask_;
};

class CameraSystem
{
  public:
  CameraSystem(const std::string& calibInfoDir, bool bPrintCalibInfo = false);
  virtual ~CameraSystem();
  using Ptr = std::shared_ptr<CameraSystem>;

  void computeBaseline();
  void loadCalibInfo(const std::string & cameraSystemDir, bool bPrintCalibInfo = false);
  void printCalibInfo();

  PerspectiveCamera::Ptr cam_left_ptr_, cam_right_ptr_; // intrinsics
  Eigen::Matrix<double, 3, 4> T_right_left_;// extrinsics
  double baseline_;
};
}
}

#endif //ESVO_CORE_CONTAINER_CAMERASYSTEM_H