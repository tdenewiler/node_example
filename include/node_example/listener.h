#ifndef NODE_EXAMPLE_LISTENER_H
#define NODE_EXAMPLE_LISTENER_H

// ROS includes.
#include "ros/ros.h"
#include "ros/time.h"

// Custom message includes. Auto-generated from msg/ directory.
#include "node_example/NodeExampleData.h"

namespace node_example
{

class ExampleListener
{
public:
  //! Constructor.
  ExampleListener(ros::NodeHandle nh);

  //! Callback function for subscriber.
  void messageCallback(const node_example::NodeExampleData::ConstPtr &msg);

private:
  //! Subscriber to custom message.
  ros::Subscriber sub_;
};

}

#endif // NODE_EXAMPLE_LISTENER_H
