#include <node_example/talker.h>

namespace node_example
{
ExampleTalker::ExampleTalker(ros::NodeHandle nh) : message_("hello"), a_(1), b_(2), enable_(true)
{
  // Set up a dynamic reconfigure server.
  // Do this before parameter server, else some of the parameter server values can be overwritten.
  dynamic_reconfigure::Server<node_example::nodeExampleConfig>::CallbackType cb;
  cb = boost::bind(&ExampleTalker::configCallback, this, _1, _2);
  dr_srv_.setCallback(cb);

  // Declare variables that can be modified by launch file or command line.
  int rate;

  // Initialize node parameters from launch file or command line. Use a private node handle so that multiple instances
  // of the node can be run simultaneously while using different parameters.
  ros::NodeHandle pnh("~");
  pnh.param("a", a_, a_);
  pnh.param("b", b_, b_);
  pnh.param("message", message_, message_);
  pnh.param("rate", rate, 1);

  // Create a publisher and name the topic.
  pub_ = nh.advertise<node_example::NodeExampleData>("example", 10);

  // Create timer.
  timer_ = nh.createTimer(ros::Duration(1 / rate), &ExampleTalker::timerCallback, this);
}

void ExampleTalker::timerCallback(const ros::TimerEvent &event)
{
  boost::unique_lock<boost::mutex> lock(mutex_);
  if (!enable_)
  {
    return;
  }

  node_example::NodeExampleData msg;
  msg.message = message_;
  msg.a = a_;
  msg.b = b_;
  lock.unlock();

  pub_.publish(msg);
}

void ExampleTalker::configCallback(node_example::nodeExampleConfig &config, uint32_t level)
{
  // Set class variables to new values. They should match what is input at the dynamic reconfigure GUI.
  boost::unique_lock<boost::mutex> lock(mutex_);
  message_ = config.message.c_str();
  a_ = config.a;
  b_ = config.b;
  enable_ = config.enable;
}
}
