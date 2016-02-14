#include "node_example/node_example.h"

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "talker");
  ros::NodeHandle nh;

  // Create a new NodeExample object.
  NodeExample *node_example = new NodeExample();

  // Set up a dynamic reconfigure server.
  // Do this before parameter server, else some of the parameter server
  // values can be overwritten.
  dynamic_reconfigure::Server<node_example::nodeExampleConfig> dr_srv;
  dynamic_reconfigure::Server<node_example::nodeExampleConfig>::CallbackType cb;
  cb = boost::bind(&NodeExample::configCallback, node_example, _1, _2);
  dr_srv.setCallback(cb);

  // Declare variables that can be modified by launch file or command line.
  int a;
  int b;
  std::string message;
  int rate;

  // Initialize node parameters from launch file or command line.
  // Use a private node handle so that multiple instances of the node can be run simultaneously
  // while using different parameters.
  ros::NodeHandle pnh("~");
  pnh.param("a", a, 1);
  pnh.param("b", b, 2);
  pnh.param("message", message, std::string("hello"));
  pnh.param("rate", rate, 40);

  // Create a publisher and name the topic.
  ros::Publisher pub_message = nh.advertise<node_example::NodeExampleData>("example", 10);

  // Tell ROS how fast to run this node.
  ros::Rate r(rate);

  // Main loop.
  while (nh.ok())
  {
    // Publish the message.
    node_example->publishMessage(&pub_message);

    ros::spinOnce();
    r.sleep();
  }

  return 0;
} // end main()
