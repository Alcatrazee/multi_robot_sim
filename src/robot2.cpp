#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf/tf.h>
#include <path_gen_srv/path_gen_srv.h>
#include <geometry_msgs/PoseStamped.h>
#include "math.h"
#include <cstdlib>
#include <time.h>

using namespace std;

#define PI 3.14159
#define max_av PI / 4
#define min_av -PI / 4
#define max_v 0.4
#define min_v -0.4

#define wait_for_command 0
#define moving 1
#define adjust 2

geometry_msgs::Twist output_cmd_vel;
path_gen_srv::path_gen_srv path_req;
size_t path_ptr = 0;
bool Get_path = false;
nav_msgs::Path path;
bool new_goal = false;
char robot_state = 0;
const float end_ore = PI / 2;
const float end_ore_tolerance = 0.5;
const float boundery = 4.8;

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
}

void odom1_callback(const nav_msgs::Odometry odom)
{

  float output_vx = 0;
  float angle_in_robot = 0;
  float target_position[2];
  float angle = get_theta(odom);
  float output_az = 0;
  float robot_target[2];
  float distance = 0;
  static float target_end_angle = PI / 2;

  switch (robot_state) // FSM
  {
  case moving:
    target_position[0] = path.poses[path_ptr].pose.position.x;
    target_position[1] = path.poses[path_ptr].pose.position.y;

    angle_in_robot = transform_world_to_robot(odom, target_position, robot_target);
    distance = sqrt(robot_target[0] * robot_target[0] + robot_target[1] * robot_target[1]);
    output_az = angle_in_robot * 2;
    output_vx = distance - abs(0.2 * output_az);
    if (output_vx > max_v)
      output_vx = max_v;
    else if (output_vx < min_v)
      output_vx = min_v;
    if (distance <= 0.4 && path_ptr > 0)
    {
      path_ptr--;
    }
    if (angle_in_robot > deg2rad(30) || angle_in_robot < -deg2rad(30))
    {
      output_vx = 0;
    }
    if (distance <= 0.025 && path_ptr == 0)
    {
      robot_state = adjust;
      if (angle > 0)
        target_end_angle = PI / 2;
      else if (angle <= 0)
        target_end_angle = -PI / 2;
      output_vx = 0;
      break;
    }
    break;
  case adjust:
    output_vx = 0;
    output_az = 5 * (target_end_angle - angle);
    if (abs(angle - target_end_angle) < deg2rad(end_ore_tolerance))
    {
      ROS_INFO("Position reached.requested position:%f,%f current_position:%f,%f,error:%f,%f angle:%f", path_req.request.goal.position.x, path_req.request.goal.position.y, odom.pose.pose.position.x, odom.pose.pose.position.y, path_req.request.goal.position.x - odom.pose.pose.position.x, path_req.request.goal.position.y - odom.pose.pose.position.y, angle);
      robot_state = wait_for_command;
      output_az = 0;
      new_goal = false;
      break;
    }
    if (output_az > max_av)
      output_az = max_av;
    else if (output_az < min_av)
      output_az = min_av;
    break;
  case wait_for_command:
    output_vx = 0;
    output_az = 0;

    break;
  }
  path_req.request.start_point.position.x = odom.pose.pose.position.x;
  path_req.request.start_point.position.y = odom.pose.pose.position.y;

  output_cmd_vel.linear.x = output_vx;
  output_cmd_vel.angular.z = output_az;
}

/* void click_callback(const geometry_msgs::PointStamped pose_des)
{
  ROS_INFO("%f %f", pose_des.point.x, pose_des.point.y);
  path_req.request.goal.position.x = pose_des.point.x;
  path_req.request.goal.position.y = pose_des.point.y;
  new_goal = true;
} */

int main(int argc, char **argv)
{
  ros::init(argc, argv, "robot2");
  ros::NodeHandle nh;
  ros::Publisher vel1_pub = nh.advertise<geometry_msgs::Twist>("robot2/cmd_vel", 1);
  ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("/robot2/path", 1);
  ros::Subscriber sub_odom1 = nh.subscribe("robot2/odom", 1, odom1_callback);
  /*   ros::Subscriber sub_point = nh.subscribe("clicked_point", 1, click_callback); */
  ros::ServiceClient client = nh.serviceClient<path_gen_srv::path_gen_srv>("/path_server");
  ros::Rate rate(100);
  geometry_msgs::Twist twist1;
  ROS_INFO("controller start!");
  ros::spinOnce();
  while (ros::ok())
  {
    if (robot_state == wait_for_command && new_goal == false)
    {
      srand(ros::Time::now().nsec);
      path_req.request.goal.position.x = (float)rand() / (float)RAND_MAX * 10 - 5;
      path_req.request.goal.position.y = (float)rand() / (float)RAND_MAX * 10 - 5;
      ROS_INFO("%f %f", path_req.request.goal.position.x, path_req.request.goal.position.y);
      if (abs(path_req.request.goal.position.x) <= boundery && abs(path_req.request.goal.position.y) < boundery)
        new_goal = true;
    }
    if (new_goal == true)
    {
      if (client.call(path_req))
      {
        robot_state = moving;
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