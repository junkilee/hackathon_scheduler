#include "ros/ros.h"
#include "hackathon_scheduler/Event.h"
#include "hackathon_scheduler/AddEvent.h"
#include "hackathon_scheduler/GetSchedule.h"
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
#include "std_srvs/Empty.h"

#include <vector>

std::vector<hackathon_scheduler::Event> schedule;

//struct for sorting events by start time
struct event_earlier_than_key
{
    inline bool operator() (const hackathon_scheduler::Event& struct1, const hackathon_scheduler::Event& struct2)
    {
        return (struct1.startTime < struct2.startTime);
    }
};

//whether two events overlap
bool overlaps(hackathon_scheduler::Event e1, hackathon_scheduler::Event e2) {
  return (e1.startTime==e2.startTime);//!(e1.endTime<=e2.startTime || e2.endTime<=e1.startTime);
}

//add an Event
bool addEvent(hackathon_scheduler::AddEvent::Request  &req,
         	 hackathon_scheduler::AddEvent::Response &res)
{
  ROS_INFO("addEvent hit");
  hackathon_scheduler::Event e = req.event;
  ROS_INFO("Attempting to add event %s of type %s with parameters %s to schedule at time %lf",e.taskName.c_str(),e.taskType.c_str(),e.parameters.c_str(),e.startTime.toSec());
  //determine if given schedule overlaps another schedule in the list
  for (std::vector<hackathon_scheduler::Event>::iterator it = schedule.begin();
       it != schedule.end(); ++it)
  {
    if (overlaps(e,*it)) {
      res.success=false;
      ROS_INFO("Events %s at %lf of type %s and %s at %lf of type %s overlap! Not adding event %s", 
          e.taskName.c_str(), e.startTime.toSec(), e.taskType.c_str(),
          (*it).taskName.c_str(), (*it).startTime.toSec(), (*it).taskType.c_str(),
          e.taskName.c_str());
      return false;
    }
  }

  schedule.push_back(e);
  ROS_INFO("Added event %s of type %s with parameters %s to schedule at time %lf",e.taskName.c_str(),e.taskType.c_str(),e.parameters.c_str(),e.startTime.toSec());
  sort(schedule.begin(), schedule.end(), event_earlier_than_key());
  ROS_INFO("Full schedule is now:");
  for (std::vector<hackathon_scheduler::Event>::iterator it = schedule.begin();
       it != schedule.end(); ++it)
  {
    ROS_INFO("    event %s of type %s with parameters {%s} at time %lf",(*it).taskName.c_str(),(*it).taskType.c_str(),(*it).parameters.c_str(),(*it).startTime.toSec());
  } 
  res.success=true;
  return true;
}

bool testAddEvent(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
  ROS_INFO("Testing the addEvent service");
  hackathon_scheduler::Event e;
  e.taskName="test";
  e.startTime=ros::Time::now()+ros::Duration(20);
  e.taskType="task1";
  e.parameters="";
  hackathon_scheduler::AddEvent::Request r;
  r.event=e;
  hackathon_scheduler::AddEvent::Response o;
  ROS_INFO("Attempting to call service");
  if (!addEvent(r,o)) ROS_INFO("Problem");
  return true;
}

//get the schedule
bool getSchedule(hackathon_scheduler::GetSchedule::Request  &req,
         	 hackathon_scheduler::GetSchedule::Response &res)
{
  res.schedule.resize(schedule.size());
  int i=0;
  for (std::vector<hackathon_scheduler::Event>::iterator it = schedule.begin();
       it != schedule.end(); ++it)
  {
    res.schedule[i++]=(*it);
  }
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "hackathon_scheduler");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("hackathon_scheduler/addEvent", addEvent);
  ros::ServiceServer testService = n.advertiseService("hackathon_scheduler/testAddEvent",testAddEvent);
  ros::ServiceServer getScheduleService = n.advertiseService("hackathon_scheduler/getSchedule",getSchedule);
  ros::Publisher publisher = n.advertise<std_msgs::String>("hackathon_scheduler/status", 100);
  ROS_INFO("Add any schedule you want.");

  ros::Rate loop_rate(1); // 1Hz

  while (ros::ok())
  {
    loop_rate.sleep();
    ros::spinOnce();
  }

  return 0;
}
