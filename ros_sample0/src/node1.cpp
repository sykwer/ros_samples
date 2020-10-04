#include <chrono>
#include <unistd.h>
#include "ros/ros.h"
#include "ros_sample0/TimestampArray.h"
#include "ros_sample0/TimestampEntry.h"

using namespace std::chrono;

class Node1 {
  ros::NodeHandle n_;
  ros::Publisher pub_;
  ros::Timer timer_;
  void timerCallback(const ros::TimerEvent &);
public:
  Node1();
};

Node1::Node1() : n_("~") {
  pub_ = n_.advertise<ros_sample0::TimestampArray>("output", 10);
  timer_ = n_.createTimer(ros::Duration(0.1), &Node1::timerCallback, this);
}

void Node1::timerCallback(const ros::TimerEvent &) {
  unsigned long long realtime_start = duration_cast<microseconds>(system_clock::now().time_since_epoch()).count();

  // Calculation from here
  usleep(10 * 1000); // 10ms
  // Calculation to here

  unsigned long long realtime_end = duration_cast<microseconds>(system_clock::now().time_since_epoch()).count();

  {
    ros_sample0::TimestampEntry start_entry;
    ros_sample0::TimestampEntry end_entry;
    start_entry.node_name = end_entry.node_name = ros::this_node::getName();
    start_entry.part_name = "timerCallback/start";
    end_entry.part_name = "timerCallback/end";
    start_entry.timestamp = realtime_start;
    end_entry.timestamp = realtime_end;

    ros_sample0::TimestampArray msg;
    msg.timestamp_entries.push_back(start_entry);
    msg.timestamp_entries.push_back(end_entry);
    pub_.publish(msg);
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "node1");
  Node1 node1;
  ros::spin();
  return 0;
}
