#include <esvo_core/core/DepthProblem.h>

namespace esvo_core
{
namespace core
{
DepthProblem::DepthProblem(
  const DepthProblemConfig::Ptr &dpConfig_ptr,
  const CameraSystem::Ptr &camSysPtr) :
  optimization::OptimizationFunctor<double>(1, 0),
  dpConfigPtr_(dpConfig_ptr),
  camSysPtr_(camSysPtr)
{

}

void DepthProblem::setProblem(
  Eigen::Vector2d & coor,
  Eigen::Matrix<double, 4, 4> & T_world_virtual,
  StampedTimeSurfaceObs* pStampedTsObs)
{
  coordinate_      = coor;
  T_world_virtual_ = T_world_virtual;
  pStampedTsObs_ = pStampedTsObs;

  vT_left_virtual_.clear();
  vT_left_virtual_.reserve(1);
  Eigen::Matrix<double,4,4> T_left_world = pStampedTsObs_->second.tr_.inverse().getTransformationMatrix();
  Eigen::Matrix<double,4,4> T_left_virtual = T_left_world * T_world_virtual_;
  vT_left_virtual_.push_back(T_left_virtual.block<3,4>(0,0));
  resetNumberValues(dpConfigPtr_->patchSize_X_ * dpConfigPtr_->patchSize_Y_);
}

int DepthProblem::operator()( const Eigen::VectorXd &x, Eigen::VectorXd & fvec ) const
{
  size_t wx = dpConfigPtr_->patchSize_X_;
  size_t wy = dpConfigPtr_->patchSize_Y_;
  size_t patchSize = wx * wy;
  int numValid  = 0;

  Eigen::Vector2d x1_s, x2_s;
  if(!warping(coordinate_, x(0), vT_left_virtual_[0], x1_s, x2_s))
  {
    if(strcmp(dpConfigPtr_->LSnorm_.c_str(), "l2") == 0)
      for(size_t i = 0; i < patchSize; i++)
        fvec[i] = 255;
    else if(strcmp(dpConfigPtr_->LSnorm_.c_str(), "zncc") == 0)
      for(size_t i = 0; i < patchSize; i++)
        fvec[i] = 2 / sqrt(patchSize);
    else if(strcmp(dpConfigPtr_->LSnorm_.c_str(), "Tdist") == 0)
      for(size_t i = 0; i < patchSize; i++)
      {
        double residual = 255;
        double weight = (dpConfigPtr_->td_nu_ + 1) / (dpConfigPtr_->td_nu_ + std::pow(residual / dpConfigPtr_->td_scale_, 2));
        fvec[i] = sqrt(weight) * residual;
      }
    else
      exit(-1);
    return numValid;
  }

  Eigen::MatrixXd tau1, tau2;
  if (patchInterpolation(pStampedTsObs_->second.TS_left_, x1_s, tau1)
    && patchInterpolation(pStampedTsObs_->second.TS_right_, x2_s, tau2))
  {
    // compute temporal residual
    if (strcmp(dpConfigPtr_->LSnorm_.c_str(), "l2") == 0)
    {
      for(size_t y = 0; y < wy; y++)
        for(size_t x = 0; x < wx; x++)
        {
          size_t index = y * wx + x;
          fvec[index] = tau1(y,x) - tau2(y,x);
        }
    }
    else if(strcmp(dpConfigPtr_->LSnorm_.c_str(), "zncc") == 0)
    {
      for(size_t y = 0; y < wy; y++)
        for(size_t x = 0; x < wx; x++)
        {
          size_t index = y * wx + x;
          double mu1, sigma1, mu2, sigma2;
          tools::meanStdDev(tau1, mu1, sigma1);
          tools::meanStdDev(tau2, mu2, sigma2);
          fvec[index] = ((tau1(y,x) - mu1) / sigma1 - (tau2(y,x) - mu2) / sigma2) / sqrt(patchSize);
        }
    }
    else if(strcmp(dpConfigPtr_->LSnorm_.c_str(), "Tdist") == 0)
    {
      std::vector<double> vResidual(patchSize);
      std::vector<double> vResidualSquared(patchSize);
      double scaleSquaredTmp1 = dpConfigPtr_->td_scaleSquared_;
      double scaleSquaredTmp2 = -1.0;
      bool first_iteration = true;
      // loop for scale until it converges
      while(fabs(scaleSquaredTmp2 - scaleSquaredTmp1) / scaleSquaredTmp1 > 0.05 || first_iteration)
      {
        if(!first_iteration)
          scaleSquaredTmp1 = scaleSquaredTmp2;

        double sum_scaleSquared = 0;
        for(size_t y = 0; y < wy; y++)
        {
          for (size_t x = 0; x < wx; x++)
          {
            size_t index = y * wx + x;
            if (first_iteration)
            {
              vResidual[index] = tau1(y, x) - tau2(y, x);
              vResidualSquared[index] = std::pow(vResidual[index], 2);
            }
            if (vResidual[index] != 0)
              sum_scaleSquared += vResidualSquared[index] * (dpConfigPtr_->td_nu_ + 1) /
                              (dpConfigPtr_->td_nu_ + vResidualSquared[index] / scaleSquaredTmp1);
          }
        }
        if(sum_scaleSquared == 0)
        {
          scaleSquaredTmp2 = dpConfigPtr_->td_scaleSquared_;
          break;
        }
        scaleSquaredTmp2 = sum_scaleSquared / patchSize;
        first_iteration = false;
      }

      // assign reweighted residual
      for(size_t y = 0; y < wy; y++)
      {
        for (size_t x = 0; x < wx; x++)
        {
          size_t index = y * wx + x;
          double weight = (dpConfigPtr_->td_nu_ + 1) / (dpConfigPtr_->td_nu_ + vResidualSquared[index] / scaleSquaredTmp2);
          fvec[index] = sqrt(weight) * vResidual[index];
        }
      }
    }
    else
      exit(-1);
    numValid = 1;
  }
  else
  {
    if(strcmp(dpConfigPtr_->LSnorm_.c_str(), "l2") == 0)
      for(size_t i = 0; i < patchSize; i++)
        fvec[i] = 255;
    else if(strcmp(dpConfigPtr_->LSnorm_.c_str(), "zncc") == 0)
      for(size_t i = 0; i < wx * wy; i++)
        fvec[i] = 2 / sqrt(patchSize);
    else if(strcmp(dpConfigPtr_->LSnorm_.c_str(), "Tdist") == 0)
      for(size_t i = 0; i < patchSize; i++)
      {
        double residual = 255;
        double weight = (dpConfigPtr_->td_nu_ + 1) / (dpConfigPtr_->td_nu_ + std::pow(residual / dpConfigPtr_->td_scale_, 2));
        fvec[i] = sqrt(weight) * residual;
      }
    else
      exit(-1);
  }
  return numValid;
}

bool DepthProblem::warping(
  const Eigen::Vector2d &x,
  double d,
  const Eigen::Matrix<double, 3, 4> &T_left_virtual,
  Eigen::Vector2d &x1_s,
  Eigen::Vector2d &x2_s) const
{
  // back-project to 3D
  Eigen::Vector3d p_rv;
  camSysPtr_->cam_left_ptr_->cam2World(x, d, p_rv);
  // transfer to left DVS coordinate
  Eigen::Vector3d p_left = T_left_virtual.block<3, 3>(0, 0) * p_rv + T_left_virtual.block<3, 1>(0, 3);
  // project onto left and right DVS image plane
  Eigen::Vector3d x1_hom = camSysPtr_->cam_left_ptr_->P_.block<3, 3>(0, 0) * p_left +
    camSysPtr_->cam_left_ptr_->P_.block<3, 1>(0, 3);
  Eigen::Vector3d x2_hom = camSysPtr_->cam_right_ptr_->P_.block<3, 3>(0, 0) * p_left +
    camSysPtr_->cam_right_ptr_->P_.block<3, 1>(0, 3);
  x1_s = x1_hom.block<2, 1>(0, 0) / x1_hom(2);
  x2_s = x2_hom.block<2, 1>(0, 0) / x2_hom(2);

  int wx = dpConfigPtr_->patchSize_X_;
  int wy = dpConfigPtr_->patchSize_Y_;
  int width  = camSysPtr_->cam_left_ptr_->width_;
  int height = camSysPtr_->cam_left_ptr_->height_;
  if (x1_s(0) < (wx - 1) / 2 || x1_s(0) > width - (wx - 1) / 2 || x1_s(1) < (wy - 1) / 2 || x1_s(1) > height - (wy - 1) / 2)
    return false;
  if (x2_s(0) < (wx - 1) / 2 || x2_s(0) > width - (wx - 1) / 2 || x2_s(1) < (wy - 1) / 2 || x2_s(1) > height - (wy - 1) / 2)
    return false;
  return true;
}

bool DepthProblem::patchInterpolation(
  const Eigen::MatrixXd &img,
  const Eigen::Vector2d &location,
  Eigen::MatrixXd &patch,
  bool debug) const
{
  int wx = dpConfigPtr_->patchSize_X_;
  int wy = dpConfigPtr_->patchSize_Y_;
  // compute SrcPatch_UpLeft coordinate and SrcPatch_DownRight coordinate
  // check patch boundary is inside img boundary
  Eigen::Vector2i SrcPatch_UpLeft, SrcPatch_DownRight;
  SrcPatch_UpLeft << floor(location[0]) - (wx - 1) / 2, floor(location[1]) - (wy - 1) / 2;
  SrcPatch_DownRight << floor(location[0]) + (wx - 1) / 2, floor(location[1]) + (wy - 1) / 2;

  if (SrcPatch_UpLeft[0] < 0 || SrcPatch_UpLeft[1] < 0)
  {
    if(debug)
    {
      LOG(INFO) << "patchInterpolation 1: " << SrcPatch_UpLeft.transpose();
    }
    return false;
  }
  if (SrcPatch_DownRight[0] >= img.cols() || SrcPatch_DownRight[1] >= img.rows())
  {
    if(debug)
    {
      LOG(INFO) << "patchInterpolation 2: " << SrcPatch_DownRight.transpose();
    }
    return false;
  }

  // compute q1 q2 q3 q4
  Eigen::Vector2d double_indices;
  double_indices << location[1], location[0];

  std::pair<int, int> lower_indices(floor(double_indices[0]), floor(double_indices[1]));
  std::pair<int, int> upper_indices(lower_indices.first + 1, lower_indices.second + 1);

  double q1 = upper_indices.second - double_indices[1];// x
  double q2 = double_indices[1] - lower_indices.second;// x
  double q3 = upper_indices.first - double_indices[0];// y
  double q4 = double_indices[0] - lower_indices.first;// y

  // extract Src patch, size (wy+1) * (wx+1)
  int wx2 = wx + 1;
  int wy2 = wy + 1;
  if (SrcPatch_UpLeft[1] + wy >= img.rows() || SrcPatch_UpLeft[0] + wx >= img.cols())
  {
    if(debug)
    {
      LOG(INFO) << "patchInterpolation 3: " << SrcPatch_UpLeft.transpose()
                << ", location: " << location.transpose()
                << ", floor(location[0]): " << floor(location[0])
                << ", (wx - 1) / 2: " << (wx - 1) / 2
                << ", ans: " << floor(location[0]) - (wx - 1) / 2
                << ", wx: " << wx << " wy: " << wy
                << ", img.row: " << img.rows() << " img.col: " << img.cols();
    }
    return false;
  }
  Eigen::MatrixXd SrcPatch = img.block(SrcPatch_UpLeft[1], SrcPatch_UpLeft[0], wy2, wx2);

  // Compute R, size (wy+1) * wx.
  Eigen::MatrixXd R;
  R = q1 * SrcPatch.block(0, 0, wy2, wx) + q2 * SrcPatch.block(0, 1, wy2, wx);

  // Compute F, size wy * wx.
  patch = q3 * R.block(0, 0, wy, wx) + q4 * R.block(1, 0, wy, wx);
  return true;
}

}// core
}// esvo_core