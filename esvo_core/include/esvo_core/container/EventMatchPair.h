#ifndef ESVO_CORE_CORE_EVENTMATCHPAIR_H
#define ESVO_CORE_CORE_EVENTMATCHPAIR_H

#include <vector>
#include <esvo_core/tools/utils.h>
#include <esvo_core/container/CameraSystem.h>
#include <esvo_core/container/TimeSurfaceObservation.h>
#include <deque>

namespace esvo_core
{
using namespace container;
using namespace tools;
namespace core
{
struct EventMatchPair
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EventMatchPair() {}

  // raw event coordinate
  Eigen::Vector2d x_left_raw_;
  // rectified_event coordinate (left, right)
  Eigen::Vector2d x_left_, x_right_;
  // timestamp
  ros::Time t_;
  // pose of virtual view (T_world_virtual)
  Transformation trans_;
  // inverse depth
  double invDepth_;
  // match cost
  double cost_;
  // gradient (left)
  double gx_, gy_;
  // disparity
  double disp_;
};
}
}

#endif //ESVO_CORE_CORE_EVENTMATCHPAIR_H
