#ifndef SR_NODE_EXAMPLE_H
#define SR_NODE_EXAMPLE_H

// ROS includes.
#include "ros/ros.h"
#include "ros/time.h"

// Custom message includes. Auto-generated from msg/ directory.
#include "node_example/NodeExampleData.h"

// Dynamic reconfigure includes.
#include <dynamic_reconfigure/server.h>
// Auto-generated from cfg/ directory.
#include <node_example/nodeExampleConfig.h>

class NodeExample
{
public:
  //! Constructor.
  NodeExample();

  //! Destructor.
  ~NodeExample();

  //! Callback function for dynamic reconfigure server.
  void configCallback(node_example::nodeExampleConfig &config, uint32_t level);

  //! Publish the message.
  void publishMessage(ros::Publisher *pub_message);

  //! Callback function for subscriber.
  void messageCallback(const node_example::NodeExampleData::ConstPtr &msg);

private:
  //! The actual message.
  std::string message_;

  //! The first integer to use in addition.
  int a_;

  //! The second integer to use in addition.
  int b_;
};

#endif // SR_NODE_EXAMPLE_H
