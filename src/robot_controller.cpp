#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf/tf.h>
#include <multiple_rb_ctrl/dynamic_path_srv.h>
#include <geometry_msgs/PoseStamped.h>
#include <time.h>
#include <cmath>
#include <multiple_rb_ctrl/occupy_grid_srv.h>
#include <multiple_rb_ctrl/instruction_srv.h>

using namespace std;

#define PI 3.14159
#define max_av PI / 2
#define min_av -PI / 2
#define max_v 0.4
#define min_v -max_v
#define close_distance 0.1

// macro of robot state
#define stop_mode 0
#define moving_forth 1
#define rotate 2
#define wait_mode 3

// req macro
#define occupy true
#define release false

// response code
const uint8_t wait = 0;
const uint8_t go_ahead = 1;
const uint8_t change_route = 2;

geometry_msgs::Twist output_cmd_vel;
multiple_rb_ctrl::dynamic_path_srv path_req;
size_t path_ptr = 0;
nav_msgs::Path path;
bool new_goal = false;
char robot_state = stop_mode;
const float end_ore = PI / 2;
const float end_ore_tolerance = 0.5;
const float boundery = 4.5;
bool order_received = false;
nav_msgs::Odometry g_odom;

ros::ServiceClient Client_grid;

uint8_t robot_id_code = 0;

uint8_t timer_counter = 0;

uint8_t blocked_time = 0;

float transform_world_to_robot(const nav_msgs::Odometry odom, const geometry_msgs::Point target_pos, float robot_coor[2])
{
  float angle = atan2(2 * (odom.pose.pose.orientation.w * odom.pose.pose.orientation.z + odom.pose.pose.orientation.x * odom.pose.pose.orientation.y), 1 - 2 * (odom.pose.pose.orientation.y * odom.pose.pose.orientation.y + odom.pose.pose.orientation.z * odom.pose.pose.orientation.z));
  robot_coor[0] = cos(angle) * target_pos.x + sin(angle) * target_pos.y - cos(angle) * odom.pose.pose.position.x - sin(angle) * odom.pose.pose.position.y;
  robot_coor[1] = -sin(angle) * target_pos.x + cos(angle) * target_pos.y + sin(angle) * odom.pose.pose.position.x - cos(angle) * odom.pose.pose.position.y;
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

// apply to occupy two grids,param1 is the grid coordinate,param2 is the operation,true is to occupy,flase is to release
uint8_t apply_for_grid_occupation(geometry_msgs::Point grid_coor[2])
{
  multiple_rb_ctrl::occupy_grid_srv req;
  req.request.applier = robot_id_code;
  req.request.operation = true;
  req.request.point_to_apply.push_back(grid_coor[0]);
  req.request.point_to_apply.push_back(grid_coor[1]);
  ROS_INFO("ask with 2 points");
  if (Client_grid.call(req))
  {
    return req.response.operation;
  }
  else
  {
    ROS_ERROR("service error.");
    return false;
  }
}

// apply to occupy a grid or release a grid,param1 is the grid coordinate,param2 is the operation,true is to occupy,flase is to release
uint8_t apply_for_grid_occupation(geometry_msgs::Point grid_coor, bool ocp_or_disocp)
{
  multiple_rb_ctrl::occupy_grid_srv req;
  req.request.applier = robot_id_code;
  req.request.operation = ocp_or_disocp;
  req.request.point_to_apply.push_back(grid_coor);
  ROS_INFO("ask to release or with one point");
  if (Client_grid.call(req))
  {
    return req.response.operation;
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
  geometry_msgs::Point current_target;
  float angle = get_theta(g_odom);
  float output_az = 0;
  float tar_in_rb_frame[2];
  float distance = 0;
  static float target_end_angle = PI / 2;
  // state changes here!
  if (order_received == true)
  {
    // set current target(global frame)
    current_target.x = path.poses[path_ptr].pose.position.x;
    current_target.y = path.poses[path_ptr].pose.position.y;
    // get current target(robot frame)
    angle_in_robot = transform_world_to_robot(g_odom, current_target, tar_in_rb_frame);
    distance = sqrt(tar_in_rb_frame[0] * tar_in_rb_frame[0] + tar_in_rb_frame[1] * tar_in_rb_frame[1]);
    /* ROS_INFO("distance: %f path_ptr:%d robot_state%d",distance,path_ptr,robot_state); */
    // FSM
    if (robot_state == moving_forth)
    {
      // set twist
      output_az = angle_in_robot;
      output_vx = distance * 3;
      // if next point is not at the front, then steer first.
      if (abs(angle_in_robot) >= deg2rad(30))
      {
        robot_state = rotate;
      }
      // request for next move
      if (distance <= close_distance && path_ptr > 0)
      {
        geometry_msgs::Point next_target;
        float next_point_angle = 0;
        next_target.x = path.poses[path_ptr - 1].pose.position.x;
        next_target.y = path.poses[path_ptr - 1].pose.position.y;
        next_point_angle = transform_world_to_robot(g_odom, next_target, tar_in_rb_frame);
        // if next point is in front
        if (abs(next_point_angle) <= deg2rad(5))
        {
          // next next point is in front
          // apply for next next point
          if (path_ptr >= 2)
          {
            geometry_msgs::Point next_target_array[2];
            next_target_array[0].x = path.poses[path_ptr - 1].pose.position.x;
            next_target_array[0].y = path.poses[path_ptr - 1].pose.position.y;
            next_target_array[1].x = path.poses[path_ptr - 2].pose.position.x;
            next_target_array[1].y = path.poses[path_ptr - 2].pose.position.y;
            uint8_t result = apply_for_grid_occupation(next_target_array);
            if (result == go_ahead)
            {
              if (path_ptr < path.poses.size() - 1)
              {
                geometry_msgs::Point last_target;
                last_target.x = path.poses[path_ptr + 1].pose.position.x;
                last_target.y = path.poses[path_ptr + 1].pose.position.y;
                apply_for_grid_occupation(last_target, release);
              }
              // application approved
              path_ptr--;
              ROS_INFO("1 apply approved ptr--");
            }
            else if (result == wait)
            {
              ROS_INFO("service blocked,waitting");
              output_vx = 0;
              output_az = 0;
              robot_state = wait_mode;
            }
            else if (result == change_route)
            {
            change_route:
              output_vx = 0;
              output_az = 0;
              robot_state = stop_mode;
            }
            // else wait till next next point avaiable
          }
          else
          {
            uint8_t result = apply_for_grid_occupation(next_target, true);
            if (result == go_ahead)
            {
              if (path_ptr < path.poses.size() - 1)
              {
                geometry_msgs::Point last_target;
                last_target.x = path.poses[path_ptr + 1].pose.position.x;
                last_target.y = path.poses[path_ptr + 1].pose.position.y;
                apply_for_grid_occupation(last_target, release);
              }
              path_ptr--;
              ROS_INFO("2 apply approved ptr--");
            }
            else if (result == wait)
            {
              ROS_INFO("service blocked,waitting");
              output_vx = 0;
              output_az = 0;
              robot_state = wait_mode;
            }
            else if (result == change_route)
            {
              output_vx = 0;
              output_az = 0;
              robot_state = wait_mode;
            }
          }
        }
      }
      if (distance <= 0.03)
      {
        // close to next point
        // end
        if (path_ptr == 0)
        {
          robot_state = stop_mode;
          output_vx = 0;
          output_az = 0;
          geometry_msgs::Point last_target;
          last_target.x = path.poses[path_ptr + 1].pose.position.x;
          last_target.y = path.poses[path_ptr + 1].pose.position.y;
          apply_for_grid_occupation(last_target, false);
          order_received = false;
        }
        else
        {
          // closed to next point and needs to change direction
          output_vx = 0;
          if (abs(g_odom.twist.twist.linear.x <= 0.01))
          {
            // there're multiple waypoints left
            if (path_ptr >= 2)
            {
              output_vx = 0;
              robot_state = rotate;
              geometry_msgs::Point next_target_arr[2];
              next_target_arr[0].x = path.poses[path_ptr - 1].pose.position.x;
              next_target_arr[0].y = path.poses[path_ptr - 1].pose.position.y;
              next_target_arr[1].x = path.poses[path_ptr - 2].pose.position.x;
              next_target_arr[1].y = path.poses[path_ptr - 2].pose.position.y;
              // apply for next point if failed , stop_mode and wait
              uint8_t result = apply_for_grid_occupation(next_target_arr);
              if (result == go_ahead)
              {
                if (path_ptr < path.poses.size() - 1)
                {
                  geometry_msgs::Point last_target;
                  last_target.x = path.poses[path_ptr + 1].pose.position.x;
                  last_target.y = path.poses[path_ptr + 1].pose.position.y;
                  apply_for_grid_occupation(last_target, release);
                }
                path_ptr--;
                ROS_INFO("3 apply approved ptr--");
              }
              else if (result == wait)
              {
                ROS_INFO("service blocked,waitting");
                output_vx = 0;
                output_az = 0;
                robot_state = wait_mode;
              }
              else if (result == change_route)
              {
                output_vx = 0;
                output_az = 0;
                robot_state = stop_mode;
              }
            }
            // only ptr 0 and 1 or only 0 left
            else
            {
              output_vx = 0;
              robot_state = rotate;
              geometry_msgs::Point next_target;
              next_target.x = path.poses[path_ptr - 1].pose.position.x;
              next_target.y = path.poses[path_ptr - 1].pose.position.y;
              uint8_t result = apply_for_grid_occupation(next_target, true);
              // apply for next point if failed , stop_mode and wait
              if (result == go_ahead)
              {
                if (path_ptr < path.poses.size() - 1)
                {
                  geometry_msgs::Point last_target;
                  last_target.x = path.poses[path_ptr + 1].pose.position.x;
                  last_target.y = path.poses[path_ptr + 1].pose.position.y;
                  apply_for_grid_occupation(last_target, release);
                }
                // application approved
                path_ptr--;
                ROS_INFO("4 apply approved ptr--");
              } //wait
              else if (result == wait)
              {
                output_vx = 0;
                robot_state = stop_mode;
                ROS_INFO("service blocked,waitting");
              }
            }
          }
        }
      }
    }
    // robot state = rotate
    else if (robot_state == rotate)
    {
      output_vx = 0;
      output_az = angle_in_robot * 2;
      if (abs(angle_in_robot) <= deg2rad(1))
        robot_state = moving_forth;
    }
    // robot state = wait_mode
    else if (robot_state == wait_mode)
    {
      output_vx = 0;
      output_az = 0;
      // there're multiple waypoints left
      if (path_ptr >= 2)
      {
        geometry_msgs::Point next_target_arr[2];
        next_target_arr[0].x = path.poses[path_ptr - 1].pose.position.x;
        next_target_arr[0].y = path.poses[path_ptr - 1].pose.position.y;
        next_target_arr[1].x = path.poses[path_ptr - 2].pose.position.x;
        next_target_arr[1].y = path.poses[path_ptr - 2].pose.position.y;
        ROS_INFO("waiting");
        uint8_t result = apply_for_grid_occupation(next_target_arr);
        if (result == go_ahead)
        {
          if (path_ptr < path.poses.size() - 1)
          {
            geometry_msgs::Point last_target;
            last_target.x = path.poses[path_ptr + 1].pose.position.x;
            last_target.y = path.poses[path_ptr + 1].pose.position.y;
            apply_for_grid_occupation(last_target, release);
          }
          path_ptr--;
          ROS_INFO("5 apply approved ptr--");
          robot_state = moving_forth;
        }
        else if (result == wait)
        {
          ROS_INFO("service blocked,waitting");
          output_vx = 0;
          output_az = 0;
          robot_state = wait_mode;
        }
        else if (result == change_route)
        {
          output_vx = 0;
          output_az = 0;
          robot_state = stop_mode;
        }
      }
      // only one or two waypoint left
      else
      {
        geometry_msgs::Point next_target;
        next_target.x = path.poses[path_ptr - 1].pose.position.x;
        next_target.y = path.poses[path_ptr - 1].pose.position.y;
        ROS_INFO("waiting");
        uint8_t result = apply_for_grid_occupation(next_target, true);
        if (result == go_ahead)
        {
          if (path_ptr < path.poses.size() - 1)
          {
            geometry_msgs::Point last_target;
            last_target.x = path.poses[path_ptr + 1].pose.position.x;
            last_target.y = path.poses[path_ptr + 1].pose.position.y;
            apply_for_grid_occupation(last_target, release);
          }
          path_ptr--;
          ROS_INFO("6 apply approved ptr--");
        }
        else if (result == wait)
        {
          ROS_INFO("service blocked,waitting");
          output_vx = 0;
          output_az = 0;
          robot_state = wait_mode;
        }
        else if (result == change_route)
        {
          output_vx = 0;
          output_az = 0;
          robot_state = stop_mode;
        }
      }
      if (timer_counter >= blocked_time)
      {
        new_goal = true;
        timer_counter = 0;
        ROS_INFO("done waiting...");
      }
    }
    else if (robot_state == stop_mode)
    {
      //no order received
      output_az = 0;
      output_vx = 0;
      new_goal = true;
    }
  }
  else
  {
    //no order received
    output_az = 0;
    output_vx = 0;
  }

  if (robot_state != wait_mode)
    timer_counter = 0;

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

void timer_callback(const ros::TimerEvent &)
{
  timer_counter++;
}

int main(int argc, char **argv)
{
  //exctract robot id
  string robot_code = argv[1];
  char *id = &argv[1][5];
  robot_id_code = atoi(id);

  ros::init(argc, argv, robot_code);
  ros::NodeHandle nh;
  ros::Publisher vel1_pub = nh.advertise<geometry_msgs::Twist>(robot_code + "/cmd_vel", 1);
  ros::Publisher path_pub = nh.advertise<nav_msgs::Path>(robot_code + "/path", 1);
  ros::Subscriber sub_odom1 = nh.subscribe(robot_code + "/odom", 1, odom1_callback);
  if (robot_id_code == 3)
    ros::Subscriber sub_point = nh.subscribe("clicked_point", 1, click_callback);
  ros::ServiceClient client = nh.serviceClient<multiple_rb_ctrl::dynamic_path_srv>("/path_server");
  ros::ServiceClient instruction_client = nh.serviceClient<multiple_rb_ctrl::instruction_srv>("/instruction_server");
  Client_grid = nh.serviceClient<multiple_rb_ctrl::occupy_grid_srv>("/occupy_grid", true);
  ros::Timer timer = nh.createTimer(ros::Duration(1), timer_callback);
  ros::Rate rate(100);
  geometry_msgs::Twist twist1;
  sleep(3);
  Client_grid.waitForExistence();
  client.waitForExistence();
  instruction_client.waitForExistence();
  //set original odom position based on robot id
  switch (robot_id_code)
  {
  case 1:
    g_odom.pose.pose.position.y = 2;
    blocked_time = 5;
    break;
  case 2:
    g_odom.pose.pose.position.y = 1;
    blocked_time = 7;
    break;
  case 3:
    g_odom.pose.pose.position.y = -1;
    blocked_time = 9;
    break;
  case 4:
    g_odom.pose.pose.position.y = -2;
    blocked_time = 10;
    break;
  }
  g_odom.pose.pose.position.x = 4.5;
  g_odom.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 3.14159);

  multiple_rb_ctrl::instruction_srv instruction_req;
  instruction_req.request.robot_id = robot_id_code;

  path_req.request.robot_id = robot_id_code;

  robot_state = stop_mode;
  new_goal = false;
  order_received = false;
  ROS_INFO("robot%d controller start!", robot_id_code);
  while (ros::ok())
  {
    if (robot_state == stop_mode && new_goal == false && order_received == false)
    {
      ROS_INFO("requesting new goal");
      bool succ = instruction_client.call(instruction_req);
      if (succ)
      {
        path_req.request.goal.position.x = instruction_req.response.new_goal.x;
        path_req.request.goal.position.y = instruction_req.response.new_goal.y;
        ROS_INFO("new goal received,(%f,%f)", path_req.request.goal.position.x, path_req.request.goal.position.y);
        if (abs(path_req.request.goal.position.x) <= boundery && abs(path_req.request.goal.position.y) < boundery)
        {
          ROS_INFO("New goal is good");
          new_goal = true;
        }
        else
        {
          ROS_INFO("New goal is not good.Regenerate a new goal.");
        }
      }
    }
    if (new_goal == true)
    {
      // encounter conflict and so to get a new path.
      if (path_ptr != 0)
      {
        ROS_INFO("replan...");
        geometry_msgs::Point target_to_apply;
        target_to_apply.x = path.poses[path_ptr + 1].pose.position.x;
        target_to_apply.y = path.poses[path_ptr + 1].pose.position.y;
        apply_for_grid_occupation(target_to_apply, release);
        /* target_to_apply.x = path.poses[path_ptr].pose.position.x;
        target_to_apply.y = path.poses[path_ptr].pose.position.y;
        apply_for_grid_occupation(target_to_apply, false); */
      }
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