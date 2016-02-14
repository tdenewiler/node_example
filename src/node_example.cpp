#include "node_example/node_example.h"

NodeExample::NodeExample()
{
}

NodeExample::~NodeExample()
{
}

void NodeExample::publishMessage(ros::Publisher *pub_message)
{
  node_example::NodeExampleData msg;
  msg.message = message_;
  msg.a = a_;
  msg.b = b_;

  pub_message->publish(msg);
}

void NodeExample::messageCallback(const node_example::NodeExampleData::ConstPtr &msg)
{
  message_ = msg->message;
  a_ = msg->a;
  b_ = msg->b;

  // Note that these are only set to INFO so they will print to a terminal for example purposes.
  // Typically, they should be DEBUG.
  ROS_INFO("message is %s", message_.c_str());
  ROS_INFO("sum of a + b = %d", a_ + b_);
}

void NodeExample::configCallback(node_example::nodeExampleConfig &config, uint32_t level)
{
  // Set class variables to new values. They should match what is input at the dynamic reconfigure GUI.
  message_ = config.message.c_str();
  a_ = config.a;
  b_ = config.b;
}
