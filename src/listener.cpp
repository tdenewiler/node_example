#include "node_example/node_example.h"

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "listener");
  ros::NodeHandle nh;

  // Declare variables that can be modified by launch file or command line.
  int rate;

  // Initialize node parameters from launch file or command line.
  // Use a private node handle so that multiple instances of the node can be run simultaneously
  // while using different parameters.
  ros::NodeHandle pnh("~");
  pnh.param("rate", rate, int(40));

  // Create a new NodeExample object.
  NodeExample *node_example = new NodeExample();

  // Create a subscriber.
  // Name the topic, message queue, callback function with class name, and object containing callback function.
  ros::Subscriber sub_message = nh.subscribe("example", 1000, &NodeExample::messageCallback, node_example);

  // Tell ROS how fast to run this node.
  ros::Rate r(rate);

  // Main loop.
  while (nh.ok())
  {
    ros::spinOnce();
    r.sleep();
  }

  return 0;
} // end main()
