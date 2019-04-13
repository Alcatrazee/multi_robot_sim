#include "math.h"
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <multiple_rb_ctrl/occupy_grid_srv.h>
#include <ros/ros.h>
#include <time.h>
#include <nav_msgs/Odometry.h>

using namespace std;

#define unoccupied 0
#define occupy 1
#define preoccupy 2

// response code
#define wait 0
#define go_ahead 1
#define change_route 2

ros::Subscriber odom_sub[4];
nav_msgs::Odometry odom[4];

class Occupied_grid_map
{
private:
  // occupied:1  not occupied:0   preoccupied:2

public:
  uint8_t _occupied[19][19];
  int64_t _occupied_by_who[19][19];
  // check if the grid is occupied or not or preoccupied
  uint8_t check_if_occupied(uint32_t point[2], int64_t who)
  {
    if (_occupied[point[0]][point[1]] == occupy ||
        _occupied[point[0]][point[1]] == preoccupy)
    {
      if (_occupied_by_who[point[0]][point[1]] == who)
        return unoccupied;
      else
        return _occupied[point[0]][point[1]];
    }
    // if point is occupied by the same robot, then for that robot,it's
    // not occupied
    /* if (_occupied[point[0]][point[1]] == occupy)
    {
      if (_occupied_by_who[point[0]][point[1]] == who)
        return unoccupied;
      else
        return _occupied[point[0]][point[1]];
    }
    else */
    return _occupied[point[0]][point[1]];
  }

  // check who has occupied the grid
  inline uint8_t check_occupied_by_who(uint32_t point[2])
  {
    return _occupied_by_who[point[0]][point[1]];
  }

  // operation 1: occupy operation 2: preoccupy
  bool set_occupied(uint8_t operation, uint32_t point[2], int64_t who)
  {
    if (operation == preoccupy)
    {
      _occupied[point[0]][point[1]] = preoccupy;
      _occupied_by_who[point[0]][point[1]] = who;
    }
    else if (operation == occupy)
    {
      _occupied[point[0]][point[1]] = occupy;
      _occupied_by_who[point[0]][point[1]] = who;
    }
    return true;
  }

  // set the grid to unoccupied state, a grid can only be released by it's
  // master
  bool set_disoccupy(uint32_t point[2], int64_t who)
  {
    if (_occupied_by_who[point[0]][point[1]] == who)
    {
      _occupied[point[0]][point[1]] = unoccupied;
      _occupied_by_who[point[0]][point[1]] = 0;
    }
    return true;
  }
  // construct function
  Occupied_grid_map(void)
  {
    for (uint16_t i = 0; i < 19; i++)
    {
      for (uint16_t j = 0; j < 19; j++)
      {
        _occupied[i][j] = unoccupied;
      }
    }
  }
};

bool server_callback(multiple_rb_ctrl::occupy_grid_srv::Request &req,
                     multiple_rb_ctrl::occupy_grid_srv::Response &res)
{
  // operation 1: apply to occupy
  // operation 2: apply to disoccupy
  uint32_t point[2];
  clock_t start_time, end_time;
  static Occupied_grid_map Ocg;
  // check if there's only one point transmitted
  if (req.point_to_apply.size() == 1)
  {
    point[0] = (uint32_t)(req.point_to_apply[0].y / 0.5 + 9);
    point[1] = (uint32_t)(req.point_to_apply[0].x / 0.5 + 9);
    if (req.operation == occupy)
    {
      uint8_t grid_state = Ocg.check_if_occupied(point, req.applier);
      if (grid_state == unoccupied || grid_state == preoccupy)
      {
        Ocg.set_occupied(occupy, point, req.applier);
        res.operation = go_ahead;
      }
      else
      {
        res.operation = wait;
      }
    }
    else if (req.operation == unoccupied)
    {
      Ocg.set_disoccupy(point, req.applier);
      res.operation = go_ahead;
    }
    // there're two points transmitted
  }
  else if (req.point_to_apply.size() == 2)
  {
    // 1. transform coordinate into grid coordinate
    uint32_t point_next[2], point_next_next[2];
    point_next[0] = (uint32_t)(req.point_to_apply[0].y / 0.5 + 9);
    point_next[1] = (uint32_t)(req.point_to_apply[0].x / 0.5 + 9);
    point_next_next[0] = (uint32_t)(req.point_to_apply[1].y / 0.5 + 9);
    point_next_next[1] = (uint32_t)(req.point_to_apply[1].x / 0.5 + 9);
    // 2. check points' status
    int point_next_state = Ocg.check_if_occupied(point_next, req.applier);
    int point_next_next_state =
        Ocg.check_if_occupied(point_next_next, req.applier);
    // 3. combine two status
    int point_state_combine = (point_next_state << 4) + point_next_next_state;
    // 4. chose output base on combined info
    switch (point_state_combine)
    {
    case 0x00:
      Ocg.set_occupied(occupy, point_next, req.applier);
      Ocg.set_occupied(preoccupy, point_next_next, req.applier);
      res.operation = go_ahead;
      break;
    case 0x01:
      Ocg.set_occupied(occupy, point_next, req.applier);
      /*       Ocg.set_occupied(preoccupy, point_next, req.applier); */
      res.operation = go_ahead;
      break;
    case 0x02:
      Ocg.set_occupied(occupy, point_next, req.applier);
      /*       Ocg.set_occupied(preoccupy, point_next, req.applier); */
      res.operation = go_ahead;
      break;
    case 0x10:
      res.operation = wait;
      break;
    case 0x11:
      ROS_INFO("Opposite direction conflict detected.");
      if (Ocg.check_occupied_by_who(point_next) ==
          Ocg.check_occupied_by_who(point_next_next))
        res.operation = wait;
      else
        res.operation = wait; // require changes
      break;
    case 0x12:
      res.operation = wait;
      break;
    case 0x20:
      Ocg.set_occupied(occupy, point_next, req.applier);
      Ocg.set_occupied(preoccupy, point_next_next, req.applier);
      res.operation = go_ahead;
      break;
    case 0x21:
      if (Ocg.check_occupied_by_who(point_next) ==
          Ocg.check_occupied_by_who(point_next_next))
        res.operation = change_route;
      else
        res.operation = wait; // require changes
      break;
    case 0x22:
      Ocg.set_occupied(occupy, point_next, req.applier);
      res.operation = go_ahead;
      break;
    }
  }

  // show map
  if (req.operation == occupy && req.applier == 3)
  {
    for (int row = 19; row >= 0; row--)
    {
      for (int col = 0; col < 19; col++)
      {
        printf("%3d", Ocg._occupied[row][col]);
      }
      cout << endl;
    }
    cout << endl;
    for (int row = 19; row >= 0; row--)
    {
      for (int col = 0; col < 19; col++)
      {
        printf("%3d", Ocg._occupied_by_who[row][col]);
      }
      cout << endl;
    }
    cout << endl;
  }
  return true;
}

void odom1_callback(const nav_msgs::Odometry odom1)
{
  odom[0] = odom1;
}

void odom2_callback(const nav_msgs::Odometry odom2)
{
  odom[1] = odom2;
}

void odom3_callback(const nav_msgs::Odometry odom3)
{
  odom[2] = odom3;
}

void odom4_callback(const nav_msgs::Odometry odom4)
{
  odom[3] = odom4;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "grid_authentication_node");
  ros::NodeHandle nh;
  ros::ServiceServer server =
      nh.advertiseService("/occupy_grid", server_callback);
  odom_sub[0] = nh.subscribe("robot1/odom", 1, odom1_callback);
  odom_sub[2] = nh.subscribe("robot2/odom", 1, odom2_callback);
  odom_sub[3] = nh.subscribe("robot3/odom", 1, odom3_callback);
  odom_sub[4] = nh.subscribe("robot4/odom", 1, odom4_callback);
  ros::spin();
  return 0;
}