#include <iostream>
#include <string>
#include <stdio.h>

using namespace std;

int main(int argc, char **argv)
{
  int a = 0x01;
  int b = 0x01;
  printf("%x\r\n", a + (b << 4));
  return 0;
}