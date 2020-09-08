#include <esvo_core/esvo_Mapping.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "esvo_Mapping");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  esvo_core::esvo_Mapping mapper(nh, nh_private);
  ros::spin();
  return 0;
}

