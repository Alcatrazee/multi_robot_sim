#include <cmath>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <multiple_rb_ctrl/dynamic_path_srv.h>
#include <multiple_rb_ctrl/instruction_srv.h>
#include <multiple_rb_ctrl/occupy_grid_srv.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <time.h>

using namespace std;

#define PI 3.14159
#define max_av PI / 2
#define min_av -PI / 2
#define max_v 0.4
#define min_v -max_v
#define close_distance 0.2

// macro of robot state
#define stop_mode 0
#define moving_forth 1
#define rotate 2
#define wait_mode 3

// req macro
#define occupy true
#define release false

// response code
#define wait 0
#define go_ahead 1
#define change_route 2

geometry_msgs::Twist output_cmd_vel;
multiple_rb_ctrl::dynamic_path_srv path_req;
ros::Publisher path_pub;
size_t path_ptr = 0;
nav_msgs::Path path;
bool new_goal = false;
char robot_state = stop_mode;
const float boundery = 4.5;
bool order_received = false;
nav_msgs::Odometry g_odom;
ros::ServiceClient Client_grid;
uint8_t robot_id_code = 0;
uint8_t timer_counter = 0;
uint8_t blocked_time = 0;
uint16_t counter_to_ask = 0;

// basic function
inline float rad2deg(float rad) { return rad * 180 / PI; }

inline float deg2rad(float deg) { return deg * PI / 180; }
// get euler from quaternion
inline float get_theta(const nav_msgs::Odometry odom)
{
  return atan2(
      2 * (odom.pose.pose.orientation.w * odom.pose.pose.orientation.z +
           odom.pose.pose.orientation.x * odom.pose.pose.orientation.y),
      1 - 2 * (odom.pose.pose.orientation.y * odom.pose.pose.orientation.y +
               odom.pose.pose.orientation.z * odom.pose.pose.orientation.z));
}

// transform world coordinate to robot frame
// odom is robot posture
// target pos is current target posture
float transform_world_to_robot(const nav_msgs::Odometry odom,
                               const geometry_msgs::Point target_pos,
                               float robot_coor[2])
{
  // quaternion to euler
  float angle = get_theta(odom);
  robot_coor[0] = cos(angle) * target_pos.x + sin(angle) * target_pos.y -
                  cos(angle) * odom.pose.pose.position.x -
                  sin(angle) * odom.pose.pose.position.y;
  robot_coor[1] = -sin(angle) * target_pos.x + cos(angle) * target_pos.y +
                  sin(angle) * odom.pose.pose.position.x -
                  cos(angle) * odom.pose.pose.position.y;
  return atan2(robot_coor[1], robot_coor[0]);
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

// apply to occupy two grids,param1 is the grid coordinate,param2 is the
// operation,true is to occupy,flase is to release
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

// apply to occupy a grid or release a grid,param1 is the grid coordinate,param2
// is the operation,true is to occupy,flase is to release
uint8_t apply_for_grid_occupation(geometry_msgs::Point grid_coor,
                                  bool ocp_or_disocp)
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
// odom callback function
// usage: set global odometry
void odom_callback(const nav_msgs::Odometry odom)
{
  g_odom.pose = odom.pose;
  g_odom.twist = odom.twist;
}

void process_fcn(void)
{
  float output_vx = 0; // output velocity
  float angle_in_robot =
      0; // temp var: to store current target direction relative to robot frame
  geometry_msgs::Point
      current_target;  // temp var: to store current target(index = path_ptr)
  float output_az = 0; // output angular velocity
  float
      tar_in_rb_frame[2]; // target(index = path_ptr) coordinate in robot frame
  float distance = 0;     // distance between current target and robot
  static float target_end_angle = PI / 2;

  if (order_received == true)
  {
    // set current target(global frame)
    current_target.x = path.poses[path_ptr].pose.position.x;
    current_target.y = path.poses[path_ptr].pose.position.y;
    // get current target(robot frame)
    angle_in_robot =
        transform_world_to_robot(g_odom, current_target, tar_in_rb_frame);
    distance = sqrt(tar_in_rb_frame[0] * tar_in_rb_frame[0] +
                    tar_in_rb_frame[1] * tar_in_rb_frame[1]);
    // FSM
    switch (robot_state)
    {
    case moving_forth:
    {
      // set twist
      output_az = angle_in_robot;
      output_vx = distance * 3;
      // if next point is not at the front, then steer first.
      if (abs(angle_in_robot) >= deg2rad(30))
      {
        robot_state = rotate;
        break;
      }
      // request for next move
      if (distance <= close_distance)
      {
        if (distance <= 0.03)
        {
          // close to next point
          // release last point as long as robot reaches next point
          if (path_ptr < path.poses.size() - 1)
          {
            geometry_msgs::Point last_target;
            last_target.x = path.poses[path_ptr + 1].pose.position.x;
            last_target.y = path.poses[path_ptr + 1].pose.position.y;
            apply_for_grid_occupation(last_target, release);
          }
          // close to the goal
          if (path_ptr == 0)
          {
            robot_state = stop_mode;
            output_vx = 0;
            output_az = 0;
            geometry_msgs::Point last_target;
            last_target.x = path.poses[path_ptr + 1].pose.position.x;
            last_target.y = path.poses[path_ptr + 1].pose.position.y;
            apply_for_grid_occupation(last_target, release);
            order_received = false;
            break;
          }
          else
          {
            // closed to next point and needs to change direction
            output_vx = 0;
            // stop before any further movement
            if (abs(g_odom.twist.twist.linear.x <= 0.01))
            {
              // there're multiple waypoints left
              if (path_ptr >= 2)
              {
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
                  if (path_ptr > 0)
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
                  output_vx = 0;
                  output_az = 0;
                  new_goal = true;
                  robot_state = stop_mode;
                }
              }
              // only ptr 0 and 1 or only 0 left
              else if (path_ptr > 0)
              {
                robot_state = rotate;
                geometry_msgs::Point next_target;
                next_target.x = path.poses[path_ptr - 1].pose.position.x;
                next_target.y = path.poses[path_ptr - 1].pose.position.y;
                uint8_t result = apply_for_grid_occupation(next_target, true);
                // apply for next point if failed , stop_mode and wait
                if (result == go_ahead)
                {
                  // application approved
                  // release last point as long as robot reaches next point
                  if (path_ptr < path.poses.size() - 1)
                  {
                    geometry_msgs::Point last_target;
                    last_target.x = path.poses[path_ptr + 1].pose.position.x;
                    last_target.y = path.poses[path_ptr + 1].pose.position.y;
                    apply_for_grid_occupation(last_target, release);
                  }
                  if (path_ptr > 0)
                    path_ptr--;
                  ROS_INFO("2 apply approved ptr--");
                } // wait
                else if (result == wait)
                {
                  robot_state = wait_mode;
                  ROS_INFO("service blocked,waitting");
                }
              }
            }
          }
        }
        // current target is within range to apply for new grid occupation
        // if next next point is in front
        else
        {
          geometry_msgs::Point next_target;
          float next_point_angle = 0;
          next_target.x = path.poses[path_ptr - 1].pose.position.x;
          next_target.y = path.poses[path_ptr - 1].pose.position.y;
          next_point_angle =
              transform_world_to_robot(g_odom, next_target, tar_in_rb_frame);
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
                if (path_ptr > 0)
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
                new_goal = true;
                robot_state = stop_mode;
              }
              // else wait till next next point avaiable
            }
            else if (path_ptr > 0)
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
                if (path_ptr > 0)
                  path_ptr--;
                ROS_INFO("4 apply approved ptr--");
              }
              else if (result == wait)
              {
                ROS_INFO("service blocked,waitting");
                output_vx = 0;
                output_az = 0;
                robot_state = wait_mode;
              }
            }
          }
        }
      }
      break;
    }
    case rotate:
    {
      output_vx = 0;
      output_az = angle_in_robot * 2;
      if (abs(angle_in_robot) <= deg2rad(1))
        robot_state = moving_forth;
      break;
    }
    case wait_mode:
    {
      output_vx = 0;
      output_az = 0;
      if (counter_to_ask >= 1)
      {
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
            if (path_ptr > 0)
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
            new_goal = true;
            robot_state = stop_mode;
          }
        }
        // only one or two waypoint left
        else if (path_ptr > 0)
        {
          geometry_msgs::Point next_target;
          next_target.x = path.poses[path_ptr - 1].pose.position.x;
          next_target.y = path.poses[path_ptr - 1].pose.position.y;
          ROS_INFO("waiting");
          uint8_t result = apply_for_grid_occupation(next_target, true);
          if (result == go_ahead)
          {
            if (path_ptr < path.poses.size() - 1 && path_ptr >= 0)
            {
              geometry_msgs::Point last_target;
              last_target.x = path.poses[path_ptr + 1].pose.position.x;
              last_target.y = path.poses[path_ptr + 1].pose.position.y;
              apply_for_grid_occupation(last_target, release);
            }
            if (path_ptr > 0)
              path_ptr--;
            ROS_INFO("6 apply approved ptr--");
          }
          else if (result == wait)
          {
            ROS_INFO("service blocked,waitting");
            output_vx = 0;
            output_az = 0;
          }
          else if (result == change_route)
          {
            output_vx = 0;
            output_az = 0;
            new_goal = true;
            robot_state = stop_mode;
          }
        }
        if (timer_counter >= blocked_time)
        {
          robot_state = stop_mode;
          geometry_msgs::Point target_to_apply;
          target_to_apply.x = path.poses[path_ptr + 1].pose.position.x;
          target_to_apply.y = path.poses[path_ptr + 1].pose.position.y;
          apply_for_grid_occupation(target_to_apply, release);
          // release point that it have occupied
          target_to_apply.x = path.poses[path_ptr - 1].pose.position.x;
          target_to_apply.y = path.poses[path_ptr - 1].pose.position.y;
          apply_for_grid_occupation(target_to_apply, release);
          new_goal = true;
          timer_counter = 0;
          ROS_INFO("done waiting...");
        }
        counter_to_ask = 0;
      }
      break;
    }
    case stop_mode:
    {
      // no order received
      output_az = 0;
      output_vx = 0;
      new_goal = false;
      order_received = false;
      break;
    }
    }
  }
  else
  {
    // no order received
    output_az = 0;
    output_vx = 0;
  }
  // clear timer counter
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
}

// timer callback
// usage: publish path and count how long it has been since start waiting.
void timer_callback(const ros::TimerEvent &)
{
  timer_counter++;
  if (robot_state == wait_mode)
  {
    counter_to_ask++;
  }
  else
  {
    counter_to_ask = 0;
  }
  ROS_INFO("robot%d", robot_id_code);
  ROS_INFO("path_ptr: %lld %f %f", path_ptr, path.poses[path_ptr].pose.position.x, path.poses[path_ptr].pose.position.y);
  path_pub.publish(path_req.response.Path); // publish new path
}

int main(int argc, char **argv)
{
  // exctract robot id
  string robot_code = argv[1];
  char *id = &argv[1][5];
  robot_id_code = atoi(id);
  // initialize ros node
  ros::init(argc, argv, robot_code); // init ros node
  ros::NodeHandle nh;                // declare a ros node handle
  ros::Publisher vel1_pub = nh.advertise<geometry_msgs::Twist>(
      robot_code + "/cmd_vel", 1); // declare a velocity publisher
  path_pub = nh.advertise<nav_msgs::Path>(robot_code + "/path",
                                          1); // declare a path publisher
  ros::Subscriber sub_odom1 = nh.subscribe(robot_code + "/odom", 1,
                                           odom_callback); // subscribe odometry
  ros::ServiceClient client =
      nh.serviceClient<multiple_rb_ctrl::dynamic_path_srv>(
          "/path_server"); // client of path
  ros::ServiceClient instruction_client =
      nh.serviceClient<multiple_rb_ctrl::instruction_srv>(
          "/instruction_server"); // client of new place to go
  Client_grid = nh.serviceClient<multiple_rb_ctrl::occupy_grid_srv>(
      "/occupy_grid", true); // global client for grid occupation
  ros::Timer timer = nh.createTimer(ros::Duration(1),
                                    timer_callback); // declare a timer of 1 hz
  ros::Rate rate(200);                               // make the loop 200 hz
  sleep(3);                                          // sleep 3 seconds to wait for other applications ready

  // wait for server ready
  Client_grid.waitForExistence();
  client.waitForExistence();
  instruction_client.waitForExistence();

  // set original odom position based on robot id
  switch (robot_id_code)
  {
  case 1:
    g_odom.pose.pose.position.y = 2;
    blocked_time = 5;
    break;
  case 2:
    g_odom.pose.pose.position.y = 1;
    blocked_time = 8;
    break;
  case 3:
    g_odom.pose.pose.position.y = -1;
    blocked_time = 8;
    break;
  case 4:
    g_odom.pose.pose.position.y = -2;
    blocked_time = 5;
    break;
  }
  g_odom.pose.pose.position.x =
      4.5; // all robots starts at the same x coordinate
  g_odom.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(
      0, 0, 3.14159); // with facing negative direction of y axis

  // declare a service variable
  multiple_rb_ctrl::instruction_srv instruction_req;
  instruction_req.request.robot_id = robot_id_code; // set robot id for service

  path_req.request.robot_id = robot_id_code; // set robot id for service
  // robot state init
  robot_state = stop_mode;
  new_goal = false;
  order_received = false;
  // print message
  ROS_INFO("robot%d controller start!", robot_id_code);
  // main loop
  while (ros::ok())
  {
    // only ask for new position when it reaches the goal
    if (robot_state == stop_mode && new_goal == false &&
        order_received == false)
    {
      ROS_INFO("requesting new goal");
      bool succ = instruction_client.call(instruction_req);
      if (succ)
      {
        // set new goal to call path server
        path_req.request.goal.position.x = instruction_req.response.new_goal.x;
        path_req.request.goal.position.y = instruction_req.response.new_goal.y;
        // limit the goal position within boundary
        if (abs(path_req.request.goal.position.x) <= boundery &&
            abs(path_req.request.goal.position.y) < boundery)
        {
          new_goal = true;
        }
        else
        {
          ROS_INFO("New goal is not good.Regenerate a new goal.");
        }
      }
      else
      {
        ROS_ERROR("requesting goal failed...");
      }
    }
    // new goal is ready, call path server for path
    if (new_goal == true)
    {
      // encounter conflict and so to get a new path.
      if (path_ptr != 0)
      {
        ROS_INFO("replan...");
        // release point that it occupied before
        geometry_msgs::Point target_to_apply;
        target_to_apply.x = path.poses[path_ptr + 1].pose.position.x;
        target_to_apply.y = path.poses[path_ptr + 1].pose.position.y;
        apply_for_grid_occupation(target_to_apply, release);
        // release point that it have occupied
        target_to_apply.x = path.poses[path_ptr - 1].pose.position.x;
        target_to_apply.y = path.poses[path_ptr - 1].pose.position.y;
        apply_for_grid_occupation(target_to_apply, release);
      }
      ROS_INFO("request for path");
      if (client.call(path_req))
      {
        ROS_INFO("path aquired.");
        order_received = true;
        path = path_req.response.Path;
        path_ptr = path.poses.size() - 1;
        if (path.poses[path_ptr].pose.position.x != round_coor(g_odom.pose.pose.position.x) && path.poses[path_ptr].pose.position.y != round_coor(g_odom.pose.pose.position.y))
        {
          geometry_msgs::PoseStamped point;
          point.pose.position.x = round_coor(g_odom.pose.pose.position.x);
          point.pose.position.y = round_coor(g_odom.pose.pose.position.y);
          path.poses.push_back(point);
          path_ptr++;
        }
        geometry_msgs::Point target_to_apply;
        target_to_apply.x = path.poses[path_ptr].pose.position.x;
        target_to_apply.y = path.poses[path_ptr].pose.position.y;
        apply_for_grid_occupation(target_to_apply, occupy);

        robot_state = moving_forth;
      }
      else
      {
        ROS_ERROR("Service not responding");
      }
      new_goal = false;
    }
    ros::spinOnce();                  // process messages from subscribed topic
    process_fcn();                    // main control function
    vel1_pub.publish(output_cmd_vel); // publish control signal

    rate.sleep();
  }
  return 0;
}