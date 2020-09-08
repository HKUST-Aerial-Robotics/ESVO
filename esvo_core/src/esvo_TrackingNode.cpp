#include <esvo_core/esvo_Tracking.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "esvo_Tracking");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  esvo_core::esvo_Tracking tracker(nh, nh_private);
  ros::spin();
  return 0;
}

