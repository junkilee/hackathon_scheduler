#include "ros/ros.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dummy_action_server");
  ros::NodeHandle n;

  ros::Rate loop_rate(1); // 1Hz

  while (ros::ok())
  {
    loop_rate.sleep();
    ros::spinOnce();


  }

  return 0;
}
