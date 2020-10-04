#include <chrono>
#include <unistd.h>
#include "ros/ros.h"
#include "ros_sample1/TimestampArray.h"
#include "ros_sample1/TimestampEntry.h"

using namespace std::chrono;

class Node4{
  ros::NodeHandle nh_;
  ros::Subscriber first_sub_;
  ros::Subscriber second_sub_;
  ros::Publisher pub_;

  void first_callback(const ros_sample1::TimestampArray::ConstPtr &input_msg);
  void second_callback(const ros_sample1::TimestampArray::ConstPtr &input_msg);
public:
  Node4();
};

Node4::Node4() : nh_("~") {
  first_sub_ = nh_.subscribe("first_input", 10, &Node4::first_callback, this);
  second_sub_ = nh_.subscribe("second_input", 10, &Node4::second_callback, this);
  pub_ = nh_.advertise<ros_sample1::TimestampArray>("output", 10);
}


void Node4::first_callback(const ros_sample1::TimestampArray::ConstPtr &input_msg) {
  unsigned long long realtime_start = duration_cast<microseconds>(system_clock::now().time_since_epoch()).count();

  // Calculation from here
  usleep(40 * 1000); // 40ms
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

void Node4::second_callback(const ros_sample1::TimestampArray::ConstPtr &input_msg) {
  unsigned long long realtime_start = duration_cast<microseconds>(system_clock::now().time_since_epoch()).count();

  // Calculation from here
  usleep(50 * 1000); // 50ms
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
  ros::init(argc, argv, "node4");
  Node4 node4;
  ros::spin();

  return 0;
}
