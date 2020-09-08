#ifndef ESVO_CORE_TOOLS_VISUALIZATION_H
#define ESVO_CORE_TOOLS_VISUALIZATION_H

#include <esvo_core/container/DepthMap.h>

namespace esvo_core
{
using namespace container;
namespace tools
{
enum VisMapType
{
  InvDepthMap,
  StdVarMap,
  CostMap,
  AgeMap
};
class Visualization
{
  public:
  Visualization();

  virtual ~Visualization();

  void plot_map(
    DepthMap::Ptr &depthMapPtr,
    VisMapType vmType,
    cv::Mat &img,
    double max_range,
    double min_range,
    double visualization_threshold1,
    double visualization_threshold2 = 0.0);

  void plot_eventMap(
    std::vector<dvs_msgs::Event*>& vEventPtr,
    cv::Mat & eventMap,
    size_t row, size_t col);

  void plot_events(
    std::vector<Eigen::Matrix<double,2,1>,
      Eigen::aligned_allocator<Eigen::Matrix<double,2,1> > > & vEvents,
    cv::Mat & event_img,
    size_t row, size_t col);

  void DrawPoint(
    double val,
    double max_range,
    double min_range,
    const Eigen::Vector2d &location,
    cv::Mat &img );

  public:
  //the rgb values for a jet colormap with 256 values
  static const float r[];
  static const float g[];
  static const float b[];
};
}
}

#endif //ESVO_CORE_TOOLS_VISUALIZATION_H
