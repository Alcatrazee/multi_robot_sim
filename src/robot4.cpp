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
#include <multiple_rb_ctrl/occupy_grid_srv.h>
#include <multiple_rb_ctrl/instruction_srv.h>

using namespace std;

#define PI 3.14159
#define max_av PI / 2
#define min_av -PI / 2
#define max_v 0.4
#define min_v -max_v
#define close_distance 0.25

#define stop 0
#define moving_forth 1
#define rotate 2
#define wait_mode 3

geometry_msgs::Twist output_cmd_vel;
path_gen_srv::path_gen_srv path_req;
size_t path_ptr = 0;
nav_msgs::Path path;
bool new_goal = false;
char robot_state = 0;
const float end_ore = PI / 2;
const float end_ore_tolerance = 0.5;
const float boundery = 4.5;
bool order_received = false;
nav_msgs::Odometry g_odom;

ros::ServiceClient Client_grid;

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

float round_coor(float num_to_round)
{
  float result = 0;
  int int_part = (int)num_to_round;
  float little_part = num_to_round - int_part;
  if (fabs(little_part) >= 0.25 && fabs(little_part) < 0.75)
  {
    result = num_to_round / fabs(num_to_round) * (fabs((float)int_part) + 0.5);
  }
  else if (abs(little_part) < 0.25)
  {
    result = num_to_round / fabs(num_to_round) * abs(int_part);
  }
  else if (abs(little_part) >= 0.75)
    result = num_to_round / fabs(num_to_round) * (abs(int_part) + 1);
  return result;
}

bool apply_for_grid_occupation(float grid_coor[2], bool ocp_or_disocp)
{
  multiple_rb_ctrl::occupy_grid_srv req;
  req.request.applier = 2;
  req.request.point_to_apply.x = grid_coor[0];
  req.request.point_to_apply.y = grid_coor[1];
  if (ocp_or_disocp)
    req.request.operation = true;
  else
    req.request.operation = false;
  ROS_INFO("ask");
  if (Client_grid.call(req))
  {
    if (req.response.succeed_or_not)
    {
      ROS_INFO("answered,request approved");
      return true;
    }
    else
    {
      ROS_INFO("answered,request refused.");
      return false;
    }
  }
  else
  {
    ROS_ERROR("service error.");
    return false;
  }
}

void odom1_callback(const nav_msgs::Odometry odom)
{
  g_odom.pose = odom.pose;
  g_odom.twist = odom.twist;
  path_req.request.start_point.position.x = round_coor(g_odom.pose.pose.position.x);
  path_req.request.start_point.position.y = round_coor(g_odom.pose.pose.position.y);
}

void click_callback(const geometry_msgs::PointStamped pose_des)
{
  path_req.request.goal.position.x = round_coor(pose_des.point.x);
  path_req.request.goal.position.y = round_coor(pose_des.point.y);
  ROS_INFO("%f %f %f %f", pose_des.point.x, pose_des.point.y, path_req.request.goal.position.x, path_req.request.goal.position.y);
  new_goal = true;
}

void process_fcn(void)
{
  float output_vx = 0;
  float angle_in_robot = 0;
  float target_position[2];
  float angle = get_theta(g_odom);
  float output_az = 0;
  float robot_target[2];
  float distance = 0;
  static float target_end_angle = PI / 2;
  float temp_coor[2];
  // state changes here!
  if (order_received == true)
  {
    // set current target(global frame)
    target_position[0] = path.poses[path_ptr].pose.position.x;
    target_position[1] = path.poses[path_ptr].pose.position.y;
    // get current target(robot frame)
    angle_in_robot = transform_world_to_robot(g_odom, target_position, robot_target);
    distance = sqrt(robot_target[0] * robot_target[0] + robot_target[1] * robot_target[1]);

    // FSM
    switch (robot_state)
    {
    case moving_forth:
      // set twist
      output_az = angle_in_robot;
      output_vx = distance * 1.5;
      // request for next move
      if (distance <= close_distance && path_ptr > 0)
      {
        float target_to_apply[2];
        float next_next_point_angle = 0;
        target_to_apply[0] = path.poses[path_ptr - 1].pose.position.x;
        target_to_apply[1] = path.poses[path_ptr - 1].pose.position.y;
        next_next_point_angle = transform_world_to_robot(g_odom, target_to_apply, robot_target);
        // if next point is in front
        if (abs(next_next_point_angle) <= deg2rad(5))
        {
          // next next point is in front
          // apply for next next point
          if (apply_for_grid_occupation(target_to_apply, true))
          {
            if (path_ptr < path.poses.size() - 1)
            {
              float target_to_deocc[2];
              target_to_deocc[0] = path.poses[path_ptr + 1].pose.position.x;
              target_to_deocc[1] = path.poses[path_ptr + 1].pose.position.y;
              apply_for_grid_occupation(target_to_deocc, false);
            }
            /* ROS_INFO("ask for next move succeed"); */
            // application approved
            path_ptr--;
          }
          else
          {
            /* ROS_INFO("service blocked,waitting"); */
            output_vx = 0;
            robot_state = wait_mode;
          }
          // else wait till next next point avaiable
        }
      }
      // close to next point
      if (distance <= 0.01)
      {

        // end
        if (path_ptr == 0)
        {
          robot_state = stop;
          output_vx = 0;
          output_az = 0;
          temp_coor[0] = path.poses[path_ptr + 1].pose.position.x;
          temp_coor[1] = path.poses[path_ptr + 1].pose.position.y;
          apply_for_grid_occupation(temp_coor, false);
          order_received = false;
          break;
        }
        else
        {
          // closed to next point and needs to change direction
          output_vx = -0.5;
          if (abs(g_odom.twist.twist.linear.x <= 0.1))
          {
            output_vx = 0;
            robot_state = rotate;
            float target_to_apply[2];
            target_to_apply[0] = path.poses[path_ptr - 1].pose.position.x;
            target_to_apply[1] = path.poses[path_ptr - 1].pose.position.y;
            // apply for next point if failed , stop and wait
            if (apply_for_grid_occupation(target_to_apply, true))
            {
              if (path_ptr < path.poses.size() - 1)
              {

                float target_to_deocc[2];
                target_to_deocc[0] = path.poses[path_ptr + 1].pose.position.x;
                target_to_deocc[1] = path.poses[path_ptr + 1].pose.position.y;
                apply_for_grid_occupation(target_to_deocc, false);
              }
              /* ROS_INFO("ask for next move succeed"); */
              // application approved
              path_ptr--;
            } //wait
            else
            {
              output_vx = 0;
              robot_state = wait_mode;
              /* ROS_INFO("service blocked,waitting"); */
            }
          }
        }
      }
      break;
    // robot state = rotate
    case rotate:
      output_vx = 0;
      output_az = angle_in_robot * 2;
      if (abs(angle_in_robot) <= deg2rad(1))
        robot_state = moving_forth;
      break;

    case wait_mode:
      output_vx = 0;
      output_az = 0;
      temp_coor[0] = path.poses[path_ptr - 1].pose.position.x;
      temp_coor[1] = path.poses[path_ptr - 1].pose.position.y;
      if (apply_for_grid_occupation(temp_coor, true))
      {
        if (path_ptr < path.poses.size() - 1)
        {
          float past_target[2];
          temp_coor[0] = path.poses[path_ptr + 1].pose.position.x;
          temp_coor[1] = path.poses[path_ptr + 1].pose.position.y;
          apply_for_grid_occupation(temp_coor, false);
        }
        // application approved
        path_ptr--;
        robot_state = moving_forth;
      }
      break;
    }
  }
  else
  {
    //no order received
    output_az = 0;
    output_vx = 0;
  }

  // velocity limitation
  if (output_az > max_av)
    output_az = max_av;
  else if (output_az < min_av)
    output_az = min_av;
  // angular velocity limitation
  if (output_vx > max_v)
    output_vx = max_v;
  else if (output_vx < min_v)
    output_vx = min_v;

  output_cmd_vel.linear.x = output_vx / 2;
  output_cmd_vel.angular.z = output_az;
  /* ROS_INFO("%f %f", path.poses[path_ptr].pose.position.x, path.poses[path_ptr].pose.position.y); */
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "robot4");
  ros::NodeHandle nh;
  ros::Publisher vel1_pub = nh.advertise<geometry_msgs::Twist>("robot4/cmd_vel", 1);
  ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("/robot4/path", 1);
  ros::Subscriber sub_odom1 = nh.subscribe("robot4/odom", 1, odom1_callback);
  ros::Subscriber sub_point = nh.subscribe("clicked_point", 1, click_callback);
  ros::ServiceClient client = nh.serviceClient<path_gen_srv::path_gen_srv>("/path_server");
  ros::ServiceClient instruction_client = nh.serviceClient<multiple_rb_ctrl::instruction_srv>("/instruction_server");
  Client_grid = nh.serviceClient<multiple_rb_ctrl::occupy_grid_srv>("/occupy_grid", true);
  ros::Rate rate(100);
  geometry_msgs::Twist twist1;
  while (!Client_grid)
  {
  }
  sleep(3);
  Client_grid.waitForExistence();
  client.waitForExistence();
  g_odom.pose.pose.position.x = 4.5;
  g_odom.pose.pose.position.y = -2;
  g_odom.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 3.14159);
  path_req.request.start_point.position = g_odom.pose.pose.position;

  multiple_rb_ctrl::instruction_srv instruction_req;
  instruction_req.request.robot_id = 4;

  robot_state = stop;
  new_goal = false;
  order_received = false;

  ROS_INFO("robot4 controller start!");
  while (ros::ok())
  {

    if (robot_state == stop && new_goal == false && order_received == false)
    {
      ROS_INFO("requesting new goal");
      bool succ = instruction_client.call(instruction_req);
      ROS_INFO("%d", succ);
      if (succ)
      {
        path_req.request.goal.position.x = instruction_req.response.new_goal.x;
        path_req.request.goal.position.y = instruction_req.response.new_goal.y;
        ROS_INFO("new goal received,(%f,%f)", path_req.request.goal.position.x, path_req.request.goal.position.y);
        if (abs(path_req.request.goal.position.x) <= boundery && abs(path_req.request.goal.position.y) < boundery && (path_req.request.goal.position.x != path_req.request.start_point.position.x && path_req.request.goal.position.y != path_req.request.start_point.position.y))
        {
          ROS_INFO("New goal is good");
          new_goal = true;
        }
        else
        {
          ROS_INFO("New goal is not good.Regenerate a new goal.");
        }
      }
      /* srand(clock());      //generate random goal
      path_req.request.goal.position.x = round_coor((float)rand() / (float)RAND_MAX * 10 - 5);
      path_req.request.goal.position.y = round_coor((float)rand() / (float)RAND_MAX * 10 - 5); */
    }
    if (new_goal == true)
    {
      if (client.call(path_req))
      {
        ROS_INFO("request for path");
        order_received = true;
        path = path_req.response.Path;
        path_ptr = path.poses.size() - 1;
        robot_state = moving_forth;
      }
      else
      {
        ROS_ERROR("Service not responding");
      }
      new_goal = false;
    }
    ros::spinOnce();
    process_fcn();
    vel1_pub.publish(output_cmd_vel);
    path_pub.publish(path_req.response.Path);
    rate.sleep();
  }
  return 0;
}