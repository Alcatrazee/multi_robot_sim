#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf/tf.h>
#include <path_gen_srv/path_gen_srv.h>
#include <geometry_msgs/PoseStamped.h>

using namespace std;

#define PI 3.14159
#define max_av PI
#define min_av -PI
#define max_v 0.25
#define min_v -0.25

geometry_msgs::Twist output_cmd_vel;
path_gen_srv::path_gen_srv path_req;
size_t path_ptr = 0;
bool Get_path = false;
nav_msgs::Path path;
bool new_goal = false;

float transform_world_to_robot(const nav_msgs::Odometry odom, float target_pos[2], float robot_coor[2])
{
  float angle = atan2(2 * (odom.pose.pose.orientation.w * odom.pose.pose.orientation.z + odom.pose.pose.orientation.x * odom.pose.pose.orientation.y), 1 - 2 * (odom.pose.pose.orientation.y * odom.pose.pose.orientation.y + odom.pose.pose.orientation.z * odom.pose.pose.orientation.z));
  robot_coor[0] = cos(angle) * target_pos[0] + sin(angle) * target_pos[1] - cos(angle) * odom.pose.pose.position.x - sin(angle) * odom.pose.pose.position.y;
  robot_coor[1] = -sin(angle) * target_pos[0] + cos(angle) * target_pos[1] + sin(angle) * odom.pose.pose.position.x - cos(angle) * odom.pose.pose.position.y;
  return atan2(robot_coor[1], robot_coor[0]);
}

inline float rad2deg(float rad)
{
  return rad * 180 / PI;
}

inline float deg2rad(float deg)
{
  return deg * PI / 180;
}

inline float get_theta(const nav_msgs::Odometry odom)
{
  return atan2(2 * (odom.pose.pose.orientation.w * odom.pose.pose.orientation.z + odom.pose.pose.orientation.x * odom.pose.pose.orientation.y), 1 - 2 * (odom.pose.pose.orientation.y * odom.pose.pose.orientation.y + odom.pose.pose.orientation.z * odom.pose.pose.orientation.z));
  /* return atan2(2 * (ore.w * ore.z + ore.x * ore.y), 1 - 2 * (ore.y * ore.y + ore.z * ore.z)); */
}

void odom1_callback(const nav_msgs::Odometry odom)
{

  float output_vx = 0;
  float angle_in_robot = 0;
  float target_position[2];
  if (Get_path)
  {
    target_position[0] = path.poses[path_ptr].pose.position.x;
    target_position[1] = path.poses[path_ptr].pose.position.y;

    float robot_target[2];
    angle_in_robot = transform_world_to_robot(odom, target_position, robot_target);
    float distance = sqrt(robot_target[0] * robot_target[0] + robot_target[1] * robot_target[1]);

    float output_az = angle_in_robot * 2;
    output_vx = distance - angle_in_robot * 0.025;
    if (distance <= 0.01)
    {
      output_vx = 0;
      path_ptr = 0;
      Get_path = false;
    }
    if (output_vx > max_v)
      output_vx = max_v;
    else if (output_vx < 0)
      output_vx = 0;

    if (output_az > max_av)
      output_az = max_av;
    else if (output_az < min_av)
      output_az = min_av;

    if (distance <= 0.2 && path_ptr > 0)
    {
      path_ptr--;
    }
    if (angle_in_robot > deg2rad(20) || angle_in_robot < -deg2rad(20))
    {
      output_vx = 0;
    }
  }
  else
  {
    output_vx = 0;
    angle_in_robot = 0;
  }

  path_req.request.start_point.position.x = odom.pose.pose.position.x;
  path_req.request.start_point.position.y = odom.pose.pose.position.y;

  output_cmd_vel.linear.x = output_vx;
  output_cmd_vel.angular.z = angle_in_robot * 1.5;
  ROS_INFO("%f %f %f %f %ld", output_cmd_vel.linear.x, output_cmd_vel.angular.z, target_position[0], target_position[1], path_ptr);
}

void click_callback(const geometry_msgs::PointStamped pose_des)
{
  ROS_INFO("%f %f", pose_des.point.x, pose_des.point.y);
  path_req.request.goal.position.x = pose_des.point.x;
  path_req.request.goal.position.y = pose_des.point.y;
  new_goal = true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_node");
  ros::NodeHandle nh;
  ros::Publisher vel1_pub = nh.advertise<geometry_msgs::Twist>("/robot4/cmd_vel", 1);
  ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("/robot4/path", 1);
  ros::Subscriber sub_odom1 = nh.subscribe("/robot4/odom", 1, odom1_callback);
  ros::Subscriber sub_point = nh.subscribe("/clicked_point", 1, click_callback);
  ros::ServiceClient client = nh.serviceClient<path_gen_srv::path_gen_srv>("/path_server");
  ros::Rate rate(100);
  geometry_msgs::Twist twist1;

  ROS_INFO("controller start!");

  while (ros::ok())
  {
    if (new_goal)
    {
      if (client.call(path_req))
      {
        Get_path = true;
        path = path_req.response.Path;
        path_ptr = path.poses.size() - 1;
      }
      else
      {
        ROS_ERROR("Service not responding");
      }
      new_goal = false;
    }
    vel1_pub.publish(output_cmd_vel);
    path_pub.publish(path_req.response.Path);
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}