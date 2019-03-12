#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_node");
  ros::NodeHandle nh;
  ros::Publisher vel1_pub = nh.advertise<geometry_msgs::Twist>("robot1/cmd_vel", 10);
  ros::Publisher vel2_pub = nh.advertise<geometry_msgs::Twist>("robot2/cmd_vel", 10);
  ros::Publisher vel3_pub = nh.advertise<geometry_msgs::Twist>("robot3/cmd_vel", 10);
  ros::Rate rate(100);
  geometry_msgs::Twist twist1, twist2, twist3;
  twist3.linear.x = twist2.linear.x = twist1.linear.x = 0.0;
  twist3.angular.z = twist2.angular.z = twist1.angular.z = 0.0;
  while (ros::ok())
  {
    vel1_pub.publish(twist1);
    vel2_pub.publish(twist2);
    vel3_pub.publish(twist3);
    rate.sleep();
  }
  return 0;
}