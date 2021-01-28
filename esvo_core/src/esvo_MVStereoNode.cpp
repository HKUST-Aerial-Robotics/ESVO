#include <esvo_core/esvo_MVStereo.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "esvo_MVStereo");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  esvo_core::esvo_MVStereo mvs(nh, nh_private);
  ros::spin();
  return 0;
}