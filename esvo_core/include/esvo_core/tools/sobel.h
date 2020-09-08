#ifndef ESVO_CORE_TOOLS_SOBEL_H
#define ESVO_CORE_TOOLS_SOBEL_H

#include <iostream>
#include <Eigen/Eigen>

using namespace std;
namespace esvo_core
{
namespace tools
{
class Sobel
{
public:
  Sobel(size_t kernel_size);
  virtual ~Sobel();

  double grad_x(Eigen::Matrix3d& src);
  double grad_y(Eigen::Matrix3d& src);
  void grad_xy(Eigen::Matrix3d& src, Eigen::Vector2d& grad);
  double convolve(
    const Eigen::Matrix3d& kernel,
    const Eigen::Matrix3d& src);
private:
  size_t kernel_size_;
  static const Eigen::Matrix3d sobel_3x3_x, sobel_3x3_y;
  static const Eigen::Matrix<double,5,5> sobel_5x5_x, sobel_5x5_y;
};
}
}
#endif //ESVO_CORE_TOOLS_SOBEL_H