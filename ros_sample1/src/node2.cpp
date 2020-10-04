#include <chrono>
#include <unistd.h>
#include "ros/ros.h"
#include "ros_sample1/TimestampArray.h"
#include "ros_sample1/TimestampEntry.h"

using namespace std::chrono;

class Node2 {
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  ros::Publisher pub_;

  void callback(const ros_sample1::TimestampArray::ConstPtr &input_msg);
public:
  Node2();
};

Node2::Node2() : nh_("~") {
  sub_ = nh_.subscribe("input", 10, &Node2::callback, this);
  pub_ = nh_.advertise<ros_sample1::TimestampArray>("output", 10);
}


void Node2::callback(const ros_sample1::TimestampArray::ConstPtr &input_msg) {
  unsigned long long realtime_start = duration_cast<microseconds>(system_clock::now().time_since_epoch()).count();

  // Calculation from here
  usleep(20 * 1000); // 20ms
  // Calculation to here

  unsigned long long realtime_end = duration_cast<microseconds>(system_clock::now().time_since_epoch()).count();

  {
    ros_sample1::TimestampEntry start_entry;
    ros_sample1::TimestampEntry end_entry;
    start_entry.node_name = end_entry.node_name = ros::this_node::getName();
    start_entry.part_name = "callback/start";
    end_entry.part_name = "callback/end";
    start_entry.timestamp = realtime_start;
    end_entry.timestamp = realtime_end;

    ros_sample1::TimestampArray msg = *input_msg;
    msg.timestamp_entries.push_back(start_entry);
    msg.timestamp_entries.push_back(end_entry);

    pub_.publish(msg);
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "node2");
  Node2 node2;
  ros::spin();

  return 0;
}
