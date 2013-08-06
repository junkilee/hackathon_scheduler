#include "ros/ros.h"
#include "hackathon_scheduler/Schedule.h"
#include "hackathon_scheduler/AddSchedule.h"
#include "std_msgs/String.h"

#include <vector>

std::vector<hackathon_scheduler::Schedule> schedules;

bool addSchedule(hackathon_scheduler::AddSchedule::Request  &req,
         	 hackathon_scheduler::AddSchedule::Response &res)
{
  ROS_INFO("AddSchedule is called");
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "hackathon_scheduler");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("hackathon_scheduler_service", addSchedule);
	ros::Publisher publisher = n.advertise<std_msgs::String>("hackathon_scheduler_status", 100);
  ROS_INFO("Add any schedule you want.");

  ros::Rate loop_rate(1); // 1Hz

  while (ros::ok())
  {
    loop_rate.sleep();
    ros::spinOnce();
  }

  return 0;
}
