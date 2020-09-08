#include <esvo_time_surface/TimeSurface.h>

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "esvo_time_surface");

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  esvo_time_surface::TimeSurface ts(nh, nh_private);

  ros::spin();

  return 0;
}
