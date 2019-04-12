#include <math.h>
#include <multiple_rb_ctrl/instruction_srv.h>
#include <ros/ros.h>
#include <time.h>

#define at_docking 0
#define at_working_space 1

float round_coor(float num_to_round) {
  float result = 0;
  int int_part = (int)num_to_round;
  float little_part = num_to_round - int_part;
  if (fabs(little_part) >= 0.25 && fabs(little_part) < 0.75) {
    result = num_to_round / fabs(num_to_round) * (fabs((float)int_part) + 0.5);
  } else if (abs(little_part) < 0.25) {
    result = num_to_round / fabs(num_to_round) * abs(int_part);
  } else if (abs(little_part) >= 0.75)
    result = num_to_round / fabs(num_to_round) * (abs(int_part) + 1);
  return result;
}

void generate_new_goal(uint8_t robot_id,
                       multiple_rb_ctrl::instruction_srv::Response &res) {
  float temp_goal[2];
  switch (robot_id) {
  case 1:
    srand(clock());
    temp_goal[0] = round_coor((float)rand() / (float)RAND_MAX * 10 - 5);
    temp_goal[1] = round_coor((float)rand() / (float)RAND_MAX * 10 - 5);
    while (!(temp_goal[0] >= 1.5 && temp_goal[0] <= 2.0))
      temp_goal[0] = round_coor((float)rand() / (float)RAND_MAX * 10 - 5);
    while (true) {
      temp_goal[1] = round_coor((float)rand() / (float)RAND_MAX * 10 - 5);
      if ((temp_goal[1] >= 0.5 && temp_goal[1] <= 3.5) ||
          (temp_goal[1] <= -0.5 && temp_goal[1] >= -3.5))
        break;
    }
    res.new_goal.x = temp_goal[0];
    res.new_goal.y = temp_goal[1];
    break;
  case 2:
    srand(clock());
    temp_goal[0] = round_coor((float)rand() / (float)RAND_MAX);
    temp_goal[1] = round_coor((float)rand() / (float)RAND_MAX * 10 - 5);
    while (!(temp_goal[0] >= 0 && temp_goal[0] <= 0.5)) {
      temp_goal[0] = round_coor((float)rand() / (float)RAND_MAX);
    }

    while (true) {
      temp_goal[1] = round_coor((float)rand() / (float)RAND_MAX * 10 - 5);
      if ((temp_goal[1] >= 0.5 && temp_goal[1] <= 3.5) ||
          (temp_goal[1] <= -0.5 && temp_goal[1] >= -3.5))
        break;
    }
    res.new_goal.x = temp_goal[0];
    res.new_goal.y = temp_goal[1];
    break;
  case 3:
    srand(clock());
    temp_goal[0] = round_coor((float)rand() / (float)RAND_MAX * 10 - 5);
    temp_goal[1] = round_coor((float)rand() / (float)RAND_MAX * 10 - 5);
    while (!(temp_goal[0] <= -1 && temp_goal[0] >= -1.5)) {
      temp_goal[0] = round_coor((float)rand() / (float)RAND_MAX * 10 - 5);
    }

    while (true) {
      temp_goal[1] = round_coor((float)rand() / (float)RAND_MAX * 10 - 5);
      if ((temp_goal[1] >= 0.5 && temp_goal[1] <= 3.5) ||
          (temp_goal[1] <= -0.5 && temp_goal[1] >= -3.5))
        break;
    }
    res.new_goal.x = temp_goal[0];
    res.new_goal.y = temp_goal[1];
    break;

  case 4:
    srand(clock());
    temp_goal[0] = round_coor((float)rand() / (float)RAND_MAX * 10 - 5);
    temp_goal[1] = round_coor((float)rand() / (float)RAND_MAX * 10 - 5);
    while (!(temp_goal[0] <= -2.5 && temp_goal[0] >= -3.0)) {
      temp_goal[0] = round_coor((float)rand() / (float)RAND_MAX * 10 - 5);
    }

    while (true) {
      temp_goal[1] = round_coor((float)rand() / (float)RAND_MAX * 10 - 5);
      if ((temp_goal[1] >= 0.5 && temp_goal[1] <= 3.5) ||
          (temp_goal[1] <= -0.5 && temp_goal[1] >= -3.5))
        break;
    }
    res.new_goal.x = temp_goal[0];
    res.new_goal.y = temp_goal[1];
    break;
  }
}

bool server_callback(multiple_rb_ctrl::instruction_srv::Request &req,
                     multiple_rb_ctrl::instruction_srv::Response &res) {
  static uint8_t state[4] = {at_docking, at_docking, at_docking, at_docking};
  switch (req.robot_id) {
  case 1:
    if (state[0] == at_working_space) {
      res.new_goal.x = 4.5;
      res.new_goal.y = 2;
      state[0] = at_docking;
    } else {
      generate_new_goal(req.robot_id, res);
      state[0] = at_working_space;
    }
    break;
  case 2:
    if (state[1] == at_working_space) {
      res.new_goal.x = 4.5;
      res.new_goal.y = 1;
      state[1] = at_docking;
    } else {
      generate_new_goal(req.robot_id, res);
      state[1] = at_working_space;
    }
    break;
  case 3:
    if (state[2] == at_working_space) {
      res.new_goal.x = 4.5;
      res.new_goal.y = -1;
      state[2] = at_docking;
    } else {
      generate_new_goal(req.robot_id, res);
      state[2] = at_working_space;
    }
    break;
  case 4:
    if (state[3] == at_working_space) {
      res.new_goal.x = 4.5;
      res.new_goal.y = -2;
      state[3] = at_docking;
    } else {
      generate_new_goal(req.robot_id, res);
      state[3] = at_working_space;
    }
    break;
  }
  ROS_INFO("%d", req.robot_id);
  return true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "sechduler_node");
  ros::NodeHandle nh;
  ros::ServiceServer server =
      nh.advertiseService("instruction_server", server_callback);
  ROS_INFO("instruction server ready...");
  ros::spin();
  return 0;
}