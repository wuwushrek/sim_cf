#include "crazyflie_ros/CrazyflieROS.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "crazyflie_server");

  CrazyflieServer cfserver;
  cfserver.run();

  return 0;
}
