#include <esvo_core/tools/sobel.h>

namespace esvo_core
{
namespace tools
{
Sobel::Sobel(size_t kernel_size):kernel_size_(kernel_size)
{}

Sobel::~Sobel()
{}

double Sobel::grad_x(Eigen::Matrix3d& src)
{
  return convolve(sobel_3x3_x, src) / 8;
}

double Sobel::grad_y(Eigen::Matrix3d& src)
{
  return convolve(sobel_3x3_y, src) / 8;
}

void Sobel::grad_xy(Eigen::Matrix3d& src, Eigen::Vector2d& grad)
{
  grad << grad_x(src), grad_y(src);
}

double Sobel::convolve(
  const Eigen::Matrix3d& kernel,
  const Eigen::Matrix3d& src)
{
  return kernel.cwiseProduct(src).sum();
}

const Eigen::Matrix3d Sobel::sobel_3x3_x
  = (Eigen::Matrix3d() << -1, 0, 1,
                          -2, 0, 2,
                          -1, 0, 1).finished();
const Eigen::Matrix3d Sobel::sobel_3x3_y
  = (Eigen::Matrix3d() << -1, -2, -1,
                           0,  0,  0,
                           1,  2,  1).finished();
const Eigen::Matrix<double,5,5> Sobel::sobel_5x5_x
  = (Eigen::Matrix<double,5,5>() << -5,   -4,  0,   4,  5,
                                    -8,  -10,  0,  10,  8,
                                   -10,  -20,  0,  20,  10,
                                    -8,  -10,  0,  10,  8,
                                    -5,   -4,  0,   4,  5).finished();;
const Eigen::Matrix<double,5,5> Sobel::sobel_5x5_y
  = (Eigen::Matrix<double,5,5>() << -5,  -8,  -10,  -8, -5,
                                    -4, -10,  -20, -10, -4,
                                     0,   0,    0,   0,  0,
                                     4,  10,   20,  10,  4,
                                     5,   8,   10,   8,  5).finished();
}//tools
}//esvo_core