#include "ros/ros.h"
#include "hackathon_scheduler/Event.h"
#include "hackathon_scheduler/AddEvent.h"
#include "hackathon_scheduler/GetSchedule.h"
#include "hackathon_scheduler/RemoveEvent.h"
#include "hackathon_scheduler/TaskStatus.h"
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
#include "std_srvs/Empty.h"

#include <vector>
#include <string.h>
#include <time.h>

std::vector<hackathon_scheduler::Event> schedule;
ros::Publisher* taskStatusPublisher;

//get the number of seconds since midnight from an hh:mm time string
long int secondsFromStringTime(std::string time) {
  long int hours,minutes;
  sscanf(time.c_str(),"%ld:%ld",&hours,&minutes);
  return hours*3600+minutes*60;
}

//get the current local time as an hh:mm time string
std::string getCurrentStringTime() {
  time_t rawtime;
  struct tm * timeinfo;

  time (&rawtime);
  timeinfo = localtime (&rawtime);
  char buf[5];
  sprintf(buf,"%2d:%2d",timeinfo->tm_hour,timeinfo->tm_min);
  return std::string(buf);
}

//print the whole schedule
void printSchedule() {
  ROS_INFO("Full schedule is now:");
  for (std::vector<hackathon_scheduler::Event>::iterator it = schedule.begin();
       it != schedule.end(); ++it)
  {
    ROS_INFO("    event %s of type %s with parameters {%s} at time %s",(*it).taskName.c_str(),(*it).taskType.c_str(),(*it).parameters.c_str(),(*it).startTime.c_str());
  } 
}

//struct for sorting events by start time
struct event_earlier_than_key
{
    inline bool operator() (const hackathon_scheduler::Event& struct1, const hackathon_scheduler::Event& struct2)
    {
        return (secondsFromStringTime(struct1.startTime) < secondsFromStringTime(struct2.startTime));
    }
};

//whether two events overlap
bool overlaps(hackathon_scheduler::Event e1, hackathon_scheduler::Event e2) {
  return (e1.startTime==e2.startTime);//!(e1.endTime<=e2.startTime || e2.endTime<=e1.startTime);
}

//add an Event to the schedule (and sort the schedule from earliest to latest)
bool addEvent(hackathon_scheduler::AddEvent::Request  &req,
         	 hackathon_scheduler::AddEvent::Response &res)
{
  hackathon_scheduler::Event e = req.event;
  ROS_INFO("Attempting to add event %s of type %s with parameters %s to schedule at time %s",e.taskName.c_str(),e.taskType.c_str(),e.parameters.c_str(),e.startTime.c_str());

  //determine if given event overlaps another event in the schedule
  for (std::vector<hackathon_scheduler::Event>::iterator it = schedule.begin();
       it != schedule.end(); ++it)
  {
    if (overlaps(e,*it)) {
      res.success=false;
      ROS_INFO("Events %s at %s of type %s and %s at %s of type %s overlap! Not adding event %s", 
          e.taskName.c_str(), e.startTime.c_str(), e.taskType.c_str(),
          (*it).taskName.c_str(), (*it).startTime.c_str(), (*it).taskType.c_str(),
          e.taskName.c_str());
      return true;
    }
  }

  //if no conflicts, add the event to the schedule
  schedule.push_back(e);
  ROS_INFO("Added event %s of type %s with parameters %s to schedule at time %s",e.taskName.c_str(),e.taskType.c_str(),e.parameters.c_str(),e.startTime.c_str());
  //sort the schedule from earliest to latest
  sort(schedule.begin(), schedule.end(), event_earlier_than_key());
  printSchedule();
  res.success=true;
  return true;
}

bool removeEvent(hackathon_scheduler::RemoveEvent::Request &req,
                 hackathon_scheduler::RemoveEvent::Response &res)
{
  ROS_INFO("Attempting to remove event at time %s",req.startTime.c_str());
  res.success=false;
  for (std::vector<hackathon_scheduler::Event>::iterator it = schedule.begin();
       it != schedule.end(); ++it)
  {
    if (secondsFromStringTime((*it).startTime)==secondsFromStringTime(req.startTime)) {
      res.success=true;
      ROS_INFO("Removing event %s of type %s at time %s from schedule",(*it).taskName.c_str(),(*it).taskType.c_str(),(*it).startTime.c_str());
      it=schedule.erase(it);
      break;
    }
  }
  return true;
}
//get the schedule as an array of events
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


//test for adding events, adds a task1 called test with blank parameters at the current time+20 seconds
bool testAddEvent(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
  ROS_INFO("Testing the addEvent service");
  hackathon_scheduler::Event e;
  e.taskName="test";
  e.startTime=getCurrentStringTime();//ros::Time::now()+ros::Duration(20);
  e.taskType="task1";
  e.parameters="";
  hackathon_scheduler::AddEvent::Request r;
  r.event=e;
  hackathon_scheduler::AddEvent::Response o;
  ROS_INFO("Attempting to call service");
  if (!addEvent(r,o)) ROS_INFO("Problem");
  return true;
}

//test get the current time
bool testGetTime(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
  ROS_INFO("Getting the current local time");
  std::string t = getCurrentStringTime();
  ROS_INFO("current time: %s, seconds since midnight: %ld",t.c_str(),secondsFromStringTime(t));
  return true;
}

bool testPublishStatus(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
  ROS_INFO("Preparing to publish some test statuses");
  ros::NodeHandle n;
  ros::Rate rate(1);
  hackathon_scheduler::TaskStatus status;
  status.taskName="test";
  status.status="executing";
  status.startTime=getCurrentStringTime();
  for (int i=0; i<3; i++) {
    char buf[10];
    sprintf(buf,"%i",i);
    status.message=buf;
    taskStatusPublisher->publish(status);
    rate.sleep();
  }
  status.status="success";
  status.message="finished";
  taskStatusPublisher->publish(status);
  ros::spinOnce();
  rate.sleep();
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "hackathon_scheduler");
  ros::NodeHandle n;

  ros::ServiceServer addEventService = n.advertiseService("hackathon_scheduler/addEvent", addEvent);
  //note: to remove an event from the command line, use '!!str hh:mm' for the time so it knows it's a string
  ros::ServiceServer removeEventService = n.advertiseService("hackathon_scheduler/removeEvent", removeEvent);
  ros::ServiceServer testService = n.advertiseService("hackathon_scheduler/testAddEvent",testAddEvent);
  ros::ServiceServer testGetTimeService = n.advertiseService("hackathon_scheduler/testGetTime",testGetTime);
  ros::ServiceServer getScheduleService = n.advertiseService("hackathon_scheduler/getSchedule",getSchedule);
  ros::ServiceServer testPublishStatusService = n.advertiseService("hackathon_scheduler/testPublishStatus",testPublishStatus);
  ros::Publisher p = n.advertise<hackathon_scheduler::TaskStatus>("hackathon_scheduler/status", 100);
  taskStatusPublisher = &p;
//  ros::Publisher taskStatusPublisher = n.advertise<hackathon_scheduler::TaskStatus>("hackathon_scheduler/status", 100);
  ROS_INFO("Add any schedule you want.");

  ros::Rate loop_rate(1); // 1Hz

  while (ros::ok())
  {
    loop_rate.sleep();
    ros::spinOnce();


  }

  return 0;
}
