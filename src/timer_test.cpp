#include <ros/ros.h>

ros::Timer timer;

void timer_callback(const ros::TimerEvent &)
{
  ROS_INFO("eer");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "TimerTest");
  ros::NodeHandle nh;
  timer = nh.createTimer(ros::Duration(2), timer_callback);
  ros::spin();
  return 0;
}