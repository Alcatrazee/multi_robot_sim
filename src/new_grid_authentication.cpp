#include <iostream>
#include <cstdlib>
#include "math.h"
#include <time.h>
#include <cmath>
#include <ros/ros.h>
#include <multiple_rb_ctrl/occupy_grid_srv.h>

using namespace std;

#define unoccupied 0
#define occupy 1
#define preoccupy 2

//response code
#define wait 0
#define go_ahead 1
#define change_route 2

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
    if (_occupied[point[0]][point[1]] == occupy || _occupied[point[0]][point[1]] == preoccupy)
    {
      if (_occupied_by_who[point[0]][point[1]] == who)
        return unoccupied;
      else
        return _occupied[point[0]][point[1]];
    }
    else
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
  // set the grid to unoccupied state
  bool set_disoccupy(uint32_t point[2])
  {
    _occupied[point[0]][point[1]] = unoccupied;
    _occupied_by_who[point[0]][point[1]] = 0;
    return true;
  }
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

bool server_callback(multiple_rb_ctrl::occupy_grid_srv::Request &req, multiple_rb_ctrl::occupy_grid_srv::Response &res)
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
      Ocg.set_disoccupy(point);
      res.operation = go_ahead;
    }
  }
  else if (req.point_to_apply.size() == 2)
  {
    uint32_t point_next[2], point_next_next[2];
    point_next[0] = (uint32_t)(req.point_to_apply[0].y / 0.5 + 9);
    point_next[1] = (uint32_t)(req.point_to_apply[0].x / 0.5 + 9);
    point_next_next[0] = (uint32_t)(req.point_to_apply[1].y / 0.5 + 9);
    point_next_next[1] = (uint32_t)(req.point_to_apply[1].x / 0.5 + 9);
    int point_next_state = Ocg.check_if_occupied(point_next, req.applier);
    int point_next_next_state = Ocg.check_if_occupied(point_next_next, req.applier);
    int point_state_combine = (point_next_state << 4) + point_next_next_state;
    if (req.applier == 3)
      ROS_INFO("%d %d %d %d %d", point_next[0], point_next[1], point_next_state, point_next_next_state, req.applier);
    // switch here
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
      if (Ocg.check_occupied_by_who(point_next) == Ocg.check_occupied_by_who(point_next_next))
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
      Ocg.set_occupied(occupy, point_next, req.applier);
      res.operation = change_route;
      break;
    case 0x22:
      Ocg.set_occupied(occupy, point_next, req.applier);
      res.operation = go_ahead;
      break;
    }
    /*     ROS_INFO("%d %d %d %d ", point_next[0], point_next[1], point_next_next[0], point_next_next[1] ); */
  }

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
  }

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "grid_authentication_node");
  ros::NodeHandle nh;
  ros::ServiceServer server = nh.advertiseService("/occupy_grid", server_callback);
  ros::spin();
  return 0;
}