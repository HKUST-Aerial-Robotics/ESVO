#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/Int32.h>
#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>

struct EventMessageEditor
{
  EventMessageEditor(
    double frequency,
    std::string & messageTopic)
    :bFirstMessage_(true)
  {
    eArray_ = dvs_msgs::EventArray();
    duration_threshold_ = 1 / frequency;
    message_topic_ = messageTopic;
  }

  void resetBuffer(ros::Time startTimeStamp)
  {
    start_time_ = startTimeStamp;
    end_time_ = ros::Time(start_time_.toSec() + duration_threshold_);
    eArray_.events.clear();
    eArray_.header.stamp = end_time_;
  }

  void resetArraySize(size_t width, size_t height)
  {
    eArray_.width  = width;
    eArray_.height = height;
  }

  void insertEvent(
    dvs_msgs::Event & e,
    rosbag::Bag* bag)
  {
    if(bFirstMessage_)
    {
      resetBuffer(e.ts);
      bFirstMessage_ = false;
    }

    if(e.ts.toSec() >= end_time_.toSec())
    {
      bag->write(message_topic_.c_str(), eArray_.header.stamp, eArray_);
      resetBuffer(end_time_);
    }
    eArray_.events.push_back(e);
  }

  // variables
  dvs_msgs::EventArray eArray_;
  double duration_threshold_;
  ros::Time start_time_, end_time_;
  bool bFirstMessage_;
  std::string message_topic_;
};


int main(int argc, char* argv[])
{
  ros::init(argc, argv, "event_message_editor");

  std::string src_bag_path(argv[1]);
  std::string dst_bag_path(argv[2]);
  rosbag::Bag bag_src, bag_dst;
  bag_src.open(src_bag_path.c_str(), rosbag::bagmode::Read);
  bag_dst.open(dst_bag_path.c_str(), rosbag::bagmode::Write);

  if(!bag_src.isOpen())
  {
    ROS_INFO("No rosbag is found in the give path.");
    exit(-1);
  }
  else
  {
    ROS_INFO("***********Input Bag File Name ***********");
    ROS_INFO(argv[1]);
    ROS_INFO("******************************************");
  }

  if(!bag_dst.isOpen())
  {
    ROS_INFO("The dst bag is not opened.");
    exit(-1);
  }
  else
  {
    ROS_INFO("***********Output Bag File Name ***********");
    ROS_INFO(argv[2]);
    ROS_INFO("***************************");
  }

  const double frequency = 1000;

  // process events
  std::vector<std::string> topics;
  topics.push_back(std::string("/davis_left/events"));
  topics.push_back(std::string("/davis_right/events"));
  std::vector<std::string> topics_rename;
  topics_rename.push_back(std::string("/davis/left/events"));
  topics_rename.push_back(std::string("/davis/right/events"));
  for(size_t i = 0;i < topics.size(); i++)
  {
    rosbag::View view(bag_src, rosbag::TopicQuery(topics[i]));
    EventMessageEditor eArrayEditor(frequency, topics_rename[i]);

    // topic loop
    for(rosbag::MessageInstance const m: view)
    {
      dvs_msgs::EventArray::ConstPtr msg = m.instantiate<dvs_msgs::EventArray>();
      eArrayEditor.resetArraySize(msg->width, msg->height);
      // message loop
      for(const dvs_msgs::Event& e : msg->events)
      {
        eArrayEditor.insertEvent(const_cast<dvs_msgs::Event&>(e), &bag_dst);
      }
    }
  }

  bag_src.close();
  bag_dst.close();
  return 0;
}


