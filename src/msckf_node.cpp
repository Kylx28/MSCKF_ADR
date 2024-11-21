#include "MSCKF_ADR/msckf.h"
#include <iostream>
#include <stdlib.h>

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{

  ros::init(argc, argv, "msckf");

  std::cout << "msckf" << std::endl;

  // MSCKF::MSCKF MSCKF_class;

  ros::spin();
  return 0;


}
