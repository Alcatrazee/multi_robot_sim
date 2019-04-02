#include <iostream>
#include <cstdlib>
#include "math.h"
#include <time.h>
#include <cmath>
#include <ros/ros.h>
#include <multiple_rb_ctrl/occupy_grid_srv.h>

using namespace std;

#define occupy true
#define disoccupy false

class Occupied_grid_map
{
private:
  // occupied:true  not occupied:false
  bool _occupied[19][19];
  uint8_t _occupied_by_who[19][19];

public:
  bool check_if_occupied(uint32_t point[2])
  {
    if (_occupied[point[0]][point[1]])
    {
      return true;
    }
    else
    {
      return false;
    }
  }
  uint8_t check_occupied_by_who(uint32_t point[2])
  {
    return _occupied_by_who[point[0]][point[1]];
  }
  bool set_occupied(uint32_t point[2], uint8_t who)
  {
    if (_occupied[point[0]][point[1]])
    {
      return false;
    }
    else
    {
      _occupied[point[0]][point[1]] = true;
      _occupied_by_who[point[0]][point[1]] = who;
      return true;
    }
  }
  bool set_disoccupy(uint32_t point[2])
  {
    _occupied[point[0]][point[1]] = false;
    _occupied_by_who[point[0]][point[1]] = 0;
    return true;
  }
  Occupied_grid_map(void)
  {
    for (uint16_t i = 0; i < 19; i++)
    {
      for (uint16_t j = 0; j < 19; j++)
      {
        _occupied[i][j] = false;
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
  point[0] = (uint32_t)(req.point_to_apply.y / 0.5 + 9);
  point[1] = (uint32_t)(req.point_to_apply.x / 0.5 + 9);
  ROS_INFO("received request");
  start_time = clock();
  if (req.operation == occupy)
  {
    if (!Ocg.check_if_occupied(point))
    {
      Ocg.set_occupied(point, req.applier);
      res.succeed_or_not = true;
    }
    else
    {
      res.succeed_or_not = false;
    }
  }
  else if (req.operation == disoccupy)
  {
    Ocg.set_disoccupy(point);
    res.succeed_or_not = true;
  }
  end_time = clock();

  ROS_INFO("answered,total runtime: %f", (double)(end_time - start_time) / CLOCKS_PER_SEC);
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "scehdule_node");
  ros::NodeHandle nh;
  ros::ServiceServer server = nh.advertiseService("/occupy_grid", server_callback);
  ros::spin();
  return 0;
}