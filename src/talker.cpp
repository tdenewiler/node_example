#include "node_example/talker.h"

namespace node_example
{

ExampleTalker::ExampleTalker(ros::NodeHandle nh)
{
  // Set up a dynamic reconfigure server.
  // Do this before parameter server, else some of the parameter server values can be overwritten.
  dynamic_reconfigure::Server<node_example::nodeExampleConfig>::CallbackType cb;
  cb = boost::bind(&ExampleTalker::configCallback, this, _1, _2);
  dr_srv_.setCallback(cb);

  // Declare variables that can be modified by launch file or command line.
  int a;
  int b;
  std::string message;
  int rate;

  // Initialize node parameters from launch file or command line. Use a private node handle so that multiple instances
  // of the node can be run simultaneously while using different parameters.
  ros::NodeHandle pnh("~");
  pnh.param("a", a, 1);
  pnh.param("b", b, 2);
  pnh.param("message", message, std::string("hello"));
  pnh.param("rate", rate, 40);

  // Create a publisher and name the topic.
  pub_ = nh.advertise<node_example::NodeExampleData>("example", 10);

  // Create timer.
  timer_ = nh.createTimer(ros::Duration(1 / rate), &ExampleTalker::timerCallback, this);
}

void ExampleTalker::timerCallback(const ros::TimerEvent& event)
{
  node_example::NodeExampleData msg;
  msg.message = message_;
  msg.a = a_;
  msg.b = b_;

  pub_.publish(msg);
}

void ExampleTalker::configCallback(node_example::nodeExampleConfig& config, uint32_t level)
{
  // Set class variables to new values. They should match what is input at the dynamic reconfigure GUI.
  message_ = config.message.c_str();
  a_ = config.a;
  b_ = config.b;
}

}
