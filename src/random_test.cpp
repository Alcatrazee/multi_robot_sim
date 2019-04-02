#include <cstdlib>
#include "math.h"
#include <time.h>
#include <iostream>
#include "stdio.h"
#include <cmath>

using namespace std;

float round_coor(float num_to_round)
{
  float result = 0;
  int int_part = (int)num_to_round;
  float little_part = num_to_round - int_part;
  if (abs(little_part) >= 0.25 && abs(little_part) < 0.75)
  {
    result = num_to_round / abs(num_to_round) * (abs((float)int_part) + 0.5);
  }
  else if (abs(little_part) < 0.25)
  {
    result = num_to_round / abs(num_to_round) * abs(int_part);
  }
  else if (abs(little_part) >= 0.75)
    result = num_to_round / abs(num_to_round) * (abs(int_part) + 1);
  return result;
}

int main(int argc, char **argv)
{
  srand(clock());
  float a = 0.340588;
  while (true)
    cout << round_coor((float)rand() / (float)RAND_MAX) << endl;
  return 0;
}