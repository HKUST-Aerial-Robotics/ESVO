#include <esvo_core/core/DepthFusion.h>
#include <thread>
#include <functional>

namespace esvo_core
{
namespace core
{
DepthFusion::DepthFusion(
  CameraSystem::Ptr &camSysPtr,
  std::shared_ptr<DepthProblemConfig> & dpConfigPtr):
  camSysPtr_(camSysPtr),
  dpConfigPtr_(dpConfigPtr){}

DepthFusion::~DepthFusion() {}

bool
DepthFusion::propagate_one_point(
  DepthPoint &dp_prior,
  DepthPoint &dp_prop,
  Eigen::Matrix<double, 4, 4> &T_prop_prior)
{
  Eigen::Vector3d p_prop = T_prop_prior.block<3, 3>(0, 0) * dp_prior.p_cam() +
                           T_prop_prior.block<3, 1>(0, 3);

  Eigen::Vector2d x_prop;
  camSysPtr_->cam_left_ptr_->world2Cam(p_prop, x_prop);
  if (!boundaryCheck(x_prop(0), x_prop(1),
                     camSysPtr_->cam_left_ptr_->width_, camSysPtr_->cam_left_ptr_->height_))
    return false;

  // create a depth point with propagated attributes.
  size_t row = std::floor(x_prop(1));
  size_t col = std::floor(x_prop(0));
  dp_prop = DepthPoint(row, col);
  dp_prop.update_x(x_prop);

  // compute the new inverse depth
  double invDepth = 1.0 / p_prop(2);

  // compute the jacobian
  double denominator = T_prop_prior.block<1,2>(2,0) * dp_prior.p_cam().head(2) + T_prop_prior(2, 3);
  denominator /= dp_prior.p_cam()(2);
  denominator += T_prop_prior(2,2);
  double J = T_prop_prior(2,2) / pow(denominator, 2);

  // propagation
  double variance, scale2, nu;
  if(strcmp(dpConfigPtr_->LSnorm_.c_str(), "l2") == 0)
  {
    variance = J * J * dp_prior.variance();
    dp_prop.update(invDepth, variance);
  }
  else if(strcmp(dpConfigPtr_->LSnorm_.c_str(), "Tdist") == 0)
  {
    scale2 = J * J * dp_prior.scaleSquared();
    nu = dp_prior.nu();
    variance = nu / (nu - 2) * scale2;
    dp_prop.update_studentT(invDepth, scale2, variance, nu);
  }
  else
    exit(-1);

  dp_prop.update_p_cam(p_prop);
  dp_prop.residual() = dp_prior.residual();
  dp_prop.age() = dp_prior.age();
  return true;
}

int
DepthFusion::update(
  std::vector<DepthPoint> &dp_obs,
  DepthFrame::Ptr &df,
  int fusion_radius)
{
  int numFusion = 0;
  Eigen::Matrix<double, 4, 4> T_frame_world = df->T_world_frame_.inverse().getTransformationMatrix();
  for (size_t i = 0; i < dp_obs.size(); i++)
  {
    Eigen::Matrix<double, 4, 4> T_frame_obs = T_frame_world * dp_obs[i].T_world_cam();
    DepthPoint dp_prop;
    if (!propagate_one_point(dp_obs[i], dp_prop, T_frame_obs))
      continue;
//    LOG(INFO) << "Update: dp_prop.residual(): " << dp_prop.residual();
    numFusion += fusion(dp_prop, df->dMap_, fusion_radius);
  }
  return numFusion;
}

int
DepthFusion::fusion(
  DepthPoint &dp_prop,
  DepthMap::Ptr &dm,
  int fusion_radius)
{
  int numFusion = 0;
  // get neighbour pixels involved in fusion
  std::vector<std::pair<size_t, size_t> > vpCoordinate;// pair: <row, col>
  if(fusion_radius == 0)
  {
    const size_t patchSize = 4;
    vpCoordinate.reserve(patchSize);
    size_t row_topleft = dp_prop.row();
    size_t col_topleft = dp_prop.col();
    for(int dy = 0; dy <= 1; dy++)
      for(int dx = 0; dx <= 1; dx++)
        vpCoordinate.push_back(std::make_pair(row_topleft + dy, col_topleft + dx));
  }
  else
  {
    const size_t patchSize = (2*fusion_radius+1) * (2*fusion_radius+1);
    vpCoordinate.reserve(patchSize);
    size_t row_centre = dp_prop.row();
    size_t col_centre = dp_prop.col();
    for(int dy = -1; dy <= 1; dy++)
      for(int dx = -1; dx <= 1; dx++)
        vpCoordinate.push_back(std::make_pair(row_centre + dy, col_centre + dx));
  }
  // fusion
  for(size_t i = 0; i < vpCoordinate.size(); i++)
  {
    size_t row = vpCoordinate[i].first;
    size_t col = vpCoordinate[i].second;
    if(!boundaryCheck(col, row, camSysPtr_->cam_left_ptr_->width_, camSysPtr_->cam_left_ptr_->height_))
      continue;

    // case 1: non-occupied
    if (!dm->exists(row, col))
    {
      DepthPoint dp_new(row, col);
      if(strcmp(dpConfigPtr_->LSnorm_.c_str(), "l2") == 0)
        dp_new.update(dp_prop.invDepth(), dp_prop.variance());
      else if(strcmp(dpConfigPtr_->LSnorm_.c_str(), "Tdist") == 0)
      {
        dp_new.update_studentT(dp_prop.invDepth(), dp_prop.scaleSquared(), dp_prop.variance(), dp_prop.nu());
      }
      else
        exit(-1);

      dp_new.residual() = dp_prop.residual();
      Eigen::Vector3d p_cam;
      camSysPtr_->cam_left_ptr_->cam2World(dp_new.x(), dp_prop.invDepth(), p_cam);
      dp_new.update_p_cam(p_cam);

      dm->set(row, col, dp_new);
    }
    else// case 2: occupied
    {
      bool bCompatibility = false;
      if(strcmp(dpConfigPtr_->LSnorm_.c_str(), "l2") == 0)
        bCompatibility = chiSquareTest(dp_prop.invDepth(), dm->at(row, col).invDepth(),
                                       dp_prop.variance(), dm->at(row, col).variance());
      else if(strcmp(dpConfigPtr_->LSnorm_.c_str(), "Tdist") == 0)
      {
        bCompatibility = studentTCompatibleTest(
          dp_prop.invDepth(), dm->at(row, col).invDepth(),dp_prop.variance(), dm->at(row, col).variance());
      }
      else
        exit(-1);

      // case 2.1 compatible
      if (bCompatibility)
      {
        if(strcmp(dpConfigPtr_->LSnorm_.c_str(), "l2") == 0)
          dm->get(row, col).update(dp_prop.invDepth(), dp_prop.variance());
        else if(strcmp(dpConfigPtr_->LSnorm_.c_str(), "Tdist") == 0)
          dm->get(row, col).update_studentT(dp_prop.invDepth(), dp_prop.scaleSquared(), dp_prop.variance(), dp_prop.nu());
        else
          exit(-1);

        dm->get(row, col).age()++;
        dm->get(row, col).residual() = min(dm->get(row, col).residual(), dp_prop.residual());
        Eigen::Vector3d p_update;
        camSysPtr_->cam_left_ptr_->cam2World(dm->get(row, col).x(), dp_prop.invDepth(), p_update);
        dm->get(row, col).update_p_cam(p_update);
        numFusion++;
      }
      else // case 2.2 not compatible
      {
        // consider occlusion (the pixel is already assigned with a point that is closer to the camera)
        if (dm->at(row, col).invDepth() - 2 * sqrt(dm->at(row, col).variance()) > dp_prop.invDepth())
          continue;
        if (dp_prop.variance() < dm->at(row, col).variance()
            && dp_prop.residual() < dm->at(row, col).residual()) //&& other requirement? such as cost?
        {
          dm->get(row, col) = dp_prop;
        }
      }
    }
  }
  return numFusion;
}

bool
DepthFusion::boundaryCheck(
  double xcoor,
  double ycoor,
  size_t width,
  size_t height)
{
  if (xcoor < 0 || xcoor >= width || ycoor < 0 || ycoor >= height)
    return false;
  else
    return true;
}

bool
DepthFusion::chiSquareTest(
  double invD1, double invD2,
  double var1, double var2)
{
  double delta_d_squared = std::pow(invD1 - invD2, 2);
  double compatibility = delta_d_squared / var1 + delta_d_squared / var2;
  if (compatibility < 5.99)
    return true;
  else
    return false;
}

bool
DepthFusion::studentTCompatibleTest(
  double invD1, double invD2,
  double var1, double var2)
{
  double stdvar1 = sqrt(var1);
  double stdvar2 = sqrt(var2);
  double diff = fabs(invD1 - invD2);
  if(diff < 2*stdvar1 || diff < 2*stdvar2)
    return true;
  return false;
}

void
DepthFusion::naive_propagation(
  std::vector<DepthPoint> &dp_obs,
  DepthFrame::Ptr &df)
{
  Eigen::Matrix<double, 4, 4> T_frame_world = df->T_world_frame_.inverse().getTransformationMatrix();
  for (size_t i = 0; i < dp_obs.size(); i++)
  {
    Eigen::Matrix<double, 4, 4> T_frame_obs = T_frame_world * dp_obs[i].T_world_cam();
    DepthPoint dp_prop;
    if (!propagate_one_point(dp_obs[i], dp_prop, T_frame_obs))
      continue;
    // determine the four neighbouring pixels
    std::vector<std::pair<size_t, size_t> > vpCoordinate;
    const size_t patchSize = 4;
    vpCoordinate.reserve(patchSize);
    size_t row_topleft = dp_prop.row();
    size_t col_topleft = dp_prop.col();
    for(int dy = 0; dy <= 1; dy++)
      for(int dx = 0; dx <= 1; dx++)
        vpCoordinate.push_back(std::make_pair(row_topleft + dy, col_topleft + dx));

    // naive propagation
    for(size_t i = 0; i < vpCoordinate.size(); i++)
    {
      size_t row = vpCoordinate[i].first;
      size_t col = vpCoordinate[i].second;
      if(!boundaryCheck(col, row, camSysPtr_->cam_left_ptr_->width_, camSysPtr_->cam_left_ptr_->height_))
        continue;

      // case 1: non-occupied
      if (!df->dMap_->exists(row, col))
      {
        DepthPoint dp_new(row, col);
        dp_new.update(dp_prop.invDepth(), dp_prop.variance());
        dp_new.residual() = dp_prop.residual();
        dp_new.age() = dp_prop.age();
        Eigen::Vector3d p_cam;
        camSysPtr_->cam_left_ptr_->cam2World(dp_new.x(), dp_prop.invDepth(), p_cam);
        dp_new.update_p_cam(p_cam);
        df->dMap_->set(row, col, dp_new);
      }
      else// case 2: occupied
      {
        if(df->dMap_->at(row, col).invDepth() > dp_prop.invDepth()) // dp_prop is further
          continue;
        else
        {
          if ( dp_prop.residual() < df->dMap_->at(row, col).residual())
            df->dMap_->get(row, col) = dp_prop;
        }
      }
    }
  }
}

}// namespace core
}// namespace esvo_core