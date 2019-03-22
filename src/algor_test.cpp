#include <iostream>
#include <cstdlib>
#include "math.h"
#include <time.h>
#include <cmath>

using namespace std;

float round_coor(float num_to_round)
{
  float result = 0;
  int int_part = (int)num_to_round;
  float little_part = num_to_round - int_part;
  float sign = num_to_round / abs(num_to_round);
  if (abs(little_part) >= 0.25 && abs(little_part) < 0.75)
  {
    result = sign * (abs((float)int_part) + 0.5);
  }
  else if (abs(little_part) < 0.25)
  {
    result = sign * abs(int_part);
  }
  else if (abs(little_part) >= 0.75)
  {
    result = sign * (abs(int_part) + 1);
  }
  cout << sign << endl;
  return result;
}

int main()
{
  cout << round_coor(-1.11) << " " << round_coor(-4.330) << " " << round_coor(-4.8) << endl;
  return 0;
}